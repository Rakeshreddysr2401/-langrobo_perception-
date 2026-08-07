// ============================================================
//  Rover Firmware v2  —  micro-ROS2 on ESP32   (BTS7960 + encoders)
//
//  Hardware : ESP32 DevKit V1, 2x BTS7960 (IBT-2), 4x Rhino GB37 encoder DC
//             motors (12V, 180/193 RPM, 1:30 gear), differential drive
//             (2 motors per side, paralleled onto ONE BTS7960 per side).
//  Transport: WiFi UDP -> micro_ros_agent on Pi5 (192.168.1.16:8888)
//
//  Subscribes : /cmd_vel     geometry_msgs/Twist   (target body vx, wz)
//  Publishes  : /wheel_odom  nav_msgs/Odometry     (encoder odom for the EKF)
//
//  WHAT CHANGED FROM v1: v1 drove an L298N OPEN-LOOP (bang-bang, 51% PWM
//  floor, no encoders). v2 drives the BTS7960 with a per-side PID velocity
//  loop off the wheel encoders, and publishes wheel odometry so the Jetson
//  EKF (config/ekf.yaml -> odom1: /wheel_odom) can fuse it with cuVSLAM+gyro.
//
//  === WIRING (LangRobo_Wiring_Documentation_v1.pdf + HARDWARE.md) ===
//    Left  BTS7960 : RPWM=18 LPWM=19 R_EN=21 L_EN=22
//    Right BTS7960 : RPWM=23 LPWM=5  R_EN=27 L_EN=13
//    Encoders (A,B): Left-Front 34/35   Left-Rear  36/39
//                    Right-Front 32/33  Right-Rear 25/26   (ALL 4 read; per-side avg)
//    Encoder power : Blue=3V3, Black=GND, Green=C1(A), Yellow=C2(B)
//
//  === ENCODER VOLTAGE ===
//  GB37 encoders are rated 5V. We run them off the ESP32 3V3 pin so their A/B
//  outputs are already 3.3V-safe for the GPIOs (no level shifter needed). IF
//  counts are flaky at 3.3V, move Blue back to 5V and add a 5V->3.3V divider
//  (1k series + 2k to GND = 3.3V) on every Green/Yellow line.
//
//  === LIBRARIES (install once) ===
//    - micro_ros_arduino   (branch matching the Pi5 agent: jazzy)
//    - ESP32Encoder        (madhephaestus) — hardware PCNT quadrature decode
//
//  === CALIBRATE / BENCH TEST === see HARDWARE.md. Wheels OFF the ground first:
//  confirm each side spins the RIGHT way and each encoder counts the RIGHT sign
//  (watch Serial @115200), flipping the *_DIR macros until forward => +count.
// ============================================================

#include <WiFi.h>
#include <ArduinoOTA.h>
#include <ESP32Encoder.h>
#include <math.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

// ── WiFi + agent ─────────────────────────────────────────────────────────────
const char* WIFI_SSID  = "Airtel_Singireddy's";  // home AP (Jetson+Pi5 on it)
const char* WIFI_PASS  = "YOUR_PASSWORD";          // <-- fill locally, NEVER commit
const char* AGENT_IP   = "192.168.1.16";           // Pi5 wlan0 (reserve in router)
const uint16_t AGENT_PORT = 8888;
const char* OTA_HOSTNAME = "rover-esp32";
const char* OTA_PASS     = "YOUR_OTA_PASSWORD";    // <-- fill locally, NEVER commit

// ── Motor driver pins (BTS7960) ──────────────────────────────────────────────
#define L_RPWM 18
#define L_LPWM 19
#define L_REN  21
#define L_LEN  22
#define R_RPWM 23
#define R_LPWM 5
#define R_REN  27
#define R_LEN  13

// ── Encoder pins — ALL FOUR wheels (A,B per wheel) ───────────────────────────
#define ENC_LF_A 34
#define ENC_LF_B 35
#define ENC_LR_A 36   // VP — input-only
#define ENC_LR_B 39   // VN — input-only
#define ENC_RF_A 32
#define ENC_RF_B 33
#define ENC_RR_A 25
#define ENC_RR_B 26

// ── LEDC PWM ─────────────────────────────────────────────────────────────────
#define PWM_FREQ 1000      // Hz (BTS7960 handles up to ~25k; 1k is quiet enough)
#define PWM_RES  8         // 8-bit -> 0..255
#define CH_L_R   0
#define CH_L_L   1
#define CH_R_R   2
#define CH_R_L   3

// ╔════════════════════════ CALIBRATION ═══════════════════════════════════════╗
//  Measured on this rover. Wrong numbers = wrong odometry.
#define WHEEL_DIAMETER_M 0.085f   // 85 mm tyre OD
#define WHEEL_BASE_M     0.34f    // 34 cm between L<->R wheel centres
#define ENCODER_CPR      1560.0f  // Rhino GB37: 13 PPR * 4 (quad) * 30 gear = 1560
                                  //   counts per WHEEL rev (matches attachFullQuad).
#define MAX_WHEEL_VEL    0.86f    // m/s at full PWM = (193/60) * PI * 0.085
                                  //   (193 = GB37 rated RPM)
// Sign flips — set during the wheels-off bench test (see header / HARDWARE.md):
#define L_MOTOR_DIR (+1)   // +1/-1 so +cmd spins LEFT side FORWARD
#define R_MOTOR_DIR (+1)   // +1/-1 so +cmd spins RIGHT side FORWARD
// One flag per encoder: +1/-1 so forward motion => +count. Front & rear on a
// side may be mirror-mounted, so each gets its own flag.
#define ENC_LF_DIR (+1)
#define ENC_LR_DIR (+1)
#define ENC_RF_DIR (+1)
#define ENC_RR_DIR (+1)
// ╚═════════════════════════════════════════════════════════════════════════════╝

#define WHEEL_CIRC  (float)(M_PI * WHEEL_DIAMETER_M)
#define METRES_PER_COUNT (WHEEL_CIRC / ENCODER_CPR)

// ── Control loop + PID ───────────────────────────────────────────────────────
#define CONTROL_HZ      50.0f
#define CONTROL_DT      (1.0f / CONTROL_HZ)
#define CMD_TIMEOUT_MS  500          // stop if no /cmd_vel within this window
float Kp  = 1.5f;                    // duty per (m/s) error
float Ki  = 4.0f;                    // duty per (m/s*s)
float Kff = 1.0f / MAX_WHEEL_VEL;    // feedforward: target m/s -> nominal duty
#define MIN_MOVE_DUTY   0.12f        // static-friction breakaway when target != 0

// ── Agent connection state machine ──────────────────────────────────────────
enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
AgentState agentState = WAITING_AGENT;

#define EXECUTE_EVERY_N_MS(MS, X)  do {            \
    static volatile int64_t init = -1;             \
    if (init == -1) { init = uxr_millis(); }       \
    if (uxr_millis() - init > MS) { X; init = uxr_millis(); } \
} while (0)

// ── Encoders ─────────────────────────────────────────────────────────────────
ESP32Encoder encLF, encLR, encRF, encRR;
long lastLF = 0, lastLR = 0, lastRF = 0, lastRR = 0;

// ── Command + PID state ──────────────────────────────────────────────────────
volatile float targetVx = 0.0f, targetWz = 0.0f;
unsigned long lastCmdMs = 0;
float integL = 0.0f, integR = 0.0f;

// ── Odometry pose (integrated on the ESP32) ──────────────────────────────────
float odomX = 0.0f, odomY = 0.0f, odomTh = 0.0f;

// ── micro-ROS entities ───────────────────────────────────────────────────────
rcl_subscription_t cmdVelSub;
rcl_publisher_t    odomPub;
rcl_timer_t        controlTimer;
geometry_msgs__msg__Twist twistMsg;
nav_msgs__msg__Odometry   odomMsg;
rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;
bool timeSynced = false;

// ── BTS7960 drive: duty -1..+1 for one side ──────────────────────────────────
void driveSide(int chR, int chL, int dir, float duty) {
    duty *= dir;
    if (duty >  1.0f) duty =  1.0f;
    if (duty < -1.0f) duty = -1.0f;
    int pwm = (int)(fabsf(duty) * 255.0f);
    if (duty > 0.001f)      { ledcWrite(chR, pwm); ledcWrite(chL, 0);   }
    else if (duty < -0.001f){ ledcWrite(chR, 0);   ledcWrite(chL, pwm); }
    else                    { ledcWrite(chR, 0);   ledcWrite(chL, 0);   }
}

void stopMotors() {
    ledcWrite(CH_L_R, 0); ledcWrite(CH_L_L, 0);
    ledcWrite(CH_R_R, 0); ledcWrite(CH_R_L, 0);
}

// ── PID (velocity) for one side -> duty ──────────────────────────────────────
float pidStep(float target, float meas, float &integ) {
    if (fabsf(target) < 0.01f) { integ = 0.0f; return 0.0f; }  // parked: no creep/windup
    float err = target - meas;
    integ += err * CONTROL_DT;
    float ilim = 1.0f / Ki;                    // clamp so |Ki*integ| <= 1 (anti-windup)
    if (integ >  ilim) integ =  ilim;
    if (integ < -ilim) integ = -ilim;
    float out = Kff * target + Kp * err + Ki * integ;
    out += (target > 0.0f) ? MIN_MOVE_DUTY : -MIN_MOVE_DUTY;
    if (out >  1.0f) out =  1.0f;
    if (out < -1.0f) out = -1.0f;
    return out;
}

// ── Control + odom timer @ CONTROL_HZ ────────────────────────────────────────
void controlCb(rcl_timer_t* timer, int64_t /*last*/) {
    if (!timer) return;

    // 1) measure per-side velocity — AVERAGE both encoders on each side
    //    (front+rear): more resolution, less noise, survives one bad encoder.
    long cLF = (long)encLF.getCount() * ENC_LF_DIR;
    long cLR = (long)encLR.getCount() * ENC_LR_DIR;
    long cRF = (long)encRF.getCount() * ENC_RF_DIR;
    long cRR = (long)encRR.getCount() * ENC_RR_DIR;
    long dLF = cLF - lastLF;  lastLF = cLF;
    long dLR = cLR - lastLR;  lastLR = cLR;
    long dRF = cRF - lastRF;  lastRF = cRF;
    long dRR = cRR - lastRR;  lastRR = cRR;
    float distL = 0.5f * (dLF + dLR) * METRES_PER_COUNT;  // side avg, metres this tick
    float distR = 0.5f * (dRF + dRR) * METRES_PER_COUNT;
    float velL  = distL / CONTROL_DT;      // m/s
    float velR  = distR / CONTROL_DT;

    // 2) watchdog: silence from the Pi5 -> stop
    float tvx = targetVx, twz = targetWz;
    if (millis() - lastCmdMs > CMD_TIMEOUT_MS) { tvx = 0.0f; twz = 0.0f; }

    // 3) target -> per-wheel setpoints -> PID -> BTS7960
    float wTargetL = tvx - twz * WHEEL_BASE_M * 0.5f;
    float wTargetR = tvx + twz * WHEEL_BASE_M * 0.5f;
    driveSide(CH_L_R, CH_L_L, L_MOTOR_DIR, pidStep(wTargetL, velL, integL));
    driveSide(CH_R_R, CH_R_L, R_MOTOR_DIR, pidStep(wTargetR, velR, integR));

    // 4) integrate odometry from MEASURED wheel travel
    float ds  = 0.5f * (distL + distR);
    float dth = (distR - distL) / WHEEL_BASE_M;
    odomX  += ds * cosf(odomTh + 0.5f * dth);
    odomY  += ds * sinf(odomTh + 0.5f * dth);
    odomTh += dth;

    // 5) publish /wheel_odom (twist = the part the EKF fuses)
    int64_t ns = rmw_uros_epoch_nanos();
    odomMsg.header.stamp.sec     = (int32_t)(ns / 1000000000LL);
    odomMsg.header.stamp.nanosec = (uint32_t)(ns % 1000000000LL);
    odomMsg.pose.pose.position.x = odomX;
    odomMsg.pose.pose.position.y = odomY;
    odomMsg.pose.pose.orientation.z = sinf(odomTh * 0.5f);
    odomMsg.pose.pose.orientation.w = cosf(odomTh * 0.5f);
    odomMsg.twist.twist.linear.x  = ds  / CONTROL_DT;   // body vx
    odomMsg.twist.twist.angular.z = dth / CONTROL_DT;   // body vyaw
    rcl_publish(&odomPub, &odomMsg, NULL);

    // --- BENCH-TEST DEBUG (watch on Serial Monitor @115200, ~2 Hz) ---
    // Roll a wheel FORWARD by hand: its count must go UP. If it goes down,
    // flip that encoder's ENC_*_DIR. velL/velR must be + when driving forward.
    static uint16_t dbg = 0;
    if (++dbg >= 25) {
        dbg = 0;
        Serial.printf("enc LF=%ld LR=%ld RF=%ld RR=%ld | velL=%.2f velR=%.2f | tgt vx=%.2f wz=%.2f\n",
                      cLF, cLR, cRF, cRR, velL, velR, tvx, twz);
    }
}

// ── /cmd_vel callback ────────────────────────────────────────────────────────
void cmdVelCb(const void* msgIn) {
    const geometry_msgs__msg__Twist* m = (const geometry_msgs__msg__Twist*)msgIn;
    targetVx = (float)m->linear.x;
    targetWz = (float)m->angular.z;
    lastCmdMs = millis();
}

// ── Odometry message static setup (frame ids + covariance) ───────────────────
void initOdomMsg() {
    static char odom_frame[]  = "odom";
    static char base_frame[]  = "base_link";
    odomMsg.header.frame_id.data     = odom_frame;
    odomMsg.header.frame_id.size     = strlen(odom_frame);
    odomMsg.header.frame_id.capacity = sizeof(odom_frame);
    odomMsg.child_frame_id.data      = base_frame;
    odomMsg.child_frame_id.size      = strlen(base_frame);
    odomMsg.child_frame_id.capacity  = sizeof(base_frame);
    for (int i = 0; i < 36; i++) { odomMsg.pose.covariance[i] = 0.0; odomMsg.twist.covariance[i] = 0.0; }
    // small variance on what we actually measure, huge on the rest
    odomMsg.pose.covariance[0]  = 0.02;  odomMsg.pose.covariance[7]  = 0.02;  // x, y
    odomMsg.pose.covariance[14] = 1e6;   odomMsg.pose.covariance[21] = 1e6;
    odomMsg.pose.covariance[28] = 1e6;   odomMsg.pose.covariance[35] = 0.05;  // yaw
    odomMsg.twist.covariance[0]  = 0.01;                                       // vx
    odomMsg.twist.covariance[7]  = 1e6;  odomMsg.twist.covariance[14] = 1e6;
    odomMsg.twist.covariance[21] = 1e6;  odomMsg.twist.covariance[28] = 1e6;
    odomMsg.twist.covariance[35] = 0.02;                                       // vyaw
    odomMsg.pose.pose.orientation.w = 1.0;
}

// ── micro-ROS entity lifecycle ───────────────────────────────────────────────
bool createEntities() {
    allocator = rcl_get_default_allocator();
    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
    rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    if (rclc_node_init_default(&node, "rover_esp32", "", &support) != RCL_RET_OK) return false;
    if (rclc_subscription_init_default(&cmdVelSub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel") != RCL_RET_OK) return false;
    if (rclc_publisher_init_default(&odomPub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/wheel_odom") != RCL_RET_OK) return false;
    if (rclc_timer_init_default(&controlTimer, &support,
            RCL_MS_TO_NS((int)(1000.0f / CONTROL_HZ)), controlCb) != RCL_RET_OK) return false;

    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &cmdVelSub, &twistMsg, &cmdVelCb, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &controlTimer);

    // align stamps with the agent clock so robot_localization accepts them
    timeSynced = (rmw_uros_sync_session_time() == RMW_RET_OK);

    // fresh baseline so the first tick isn't a huge accumulated delta
    lastLF = (long)encLF.getCount() * ENC_LF_DIR;
    lastLR = (long)encLR.getCount() * ENC_LR_DIR;
    lastRF = (long)encRF.getCount() * ENC_RF_DIR;
    lastRR = (long)encRR.getCount() * ENC_RR_DIR;
    integL = integR = 0.0f;
    lastCmdMs = millis();
    Serial.println("[uROS] entities live — /cmd_vel sub, /wheel_odom pub");
    return true;
}

void destroyEntities() {
    rcl_subscription_fini(&cmdVelSub, &node);
    rcl_publisher_fini(&odomPub, &node);
    rcl_timer_fini(&controlTimer);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    Serial.println("[uROS] entities destroyed");
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("=== Rover ESP32 v2 (BTS7960 + encoders) starting ===");

    // BTS7960 enables — hold both HIGH so each half-bridge is active
    pinMode(L_REN, OUTPUT); pinMode(L_LEN, OUTPUT);
    pinMode(R_REN, OUTPUT); pinMode(R_LEN, OUTPUT);
    digitalWrite(L_REN, HIGH); digitalWrite(L_LEN, HIGH);
    digitalWrite(R_REN, HIGH); digitalWrite(R_LEN, HIGH);

    // PWM channels on RPWM/LPWM
    ledcSetup(CH_L_R, PWM_FREQ, PWM_RES); ledcAttachPin(L_RPWM, CH_L_R);
    ledcSetup(CH_L_L, PWM_FREQ, PWM_RES); ledcAttachPin(L_LPWM, CH_L_L);
    ledcSetup(CH_R_R, PWM_FREQ, PWM_RES); ledcAttachPin(R_RPWM, CH_R_R);
    ledcSetup(CH_R_L, PWM_FREQ, PWM_RES); ledcAttachPin(R_LPWM, CH_R_L);
    stopMotors();

    // Encoders (PCNT hardware quadrature). 34/35/36/39 are input-only with no
    // internal pull-up — enable weak pulls; add external pull-ups if the encoder
    // board is open-collector.
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encLF.attachFullQuad(ENC_LF_A, ENC_LF_B);
    encLR.attachFullQuad(ENC_LR_A, ENC_LR_B);
    encRF.attachFullQuad(ENC_RF_A, ENC_RF_B);
    encRR.attachFullQuad(ENC_RR_A, ENC_RR_B);
    encLF.clearCount(); encLR.clearCount();
    encRF.clearCount(); encRR.clearCount();

    initOdomMsg();

    Serial.printf("[WiFi] agent %s:%d\n", AGENT_IP, AGENT_PORT);
    set_microros_wifi_transports((char*)WIFI_SSID, (char*)WIFI_PASS, (char*)AGENT_IP, AGENT_PORT);
    WiFi.setSleep(false);   // modem sleep adds ~100ms latency to every cmd

    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASS);
    ArduinoOTA.onStart([]() { stopMotors(); });
    ArduinoOTA.begin();

    agentState = WAITING_AGENT;
    Serial.println("[READY] waiting for micro-ROS agent");
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
    ArduinoOTA.handle();
    switch (agentState) {
    case WAITING_AGENT:
        stopMotors();
        EXECUTE_EVERY_N_MS(1000,
            agentState = (rmw_uros_ping_agent(500, 2) == RMW_RET_OK) ? AGENT_AVAILABLE : WAITING_AGENT);
        break;
    case AGENT_AVAILABLE:
        agentState = createEntities() ? AGENT_CONNECTED : (destroyEntities(), WAITING_AGENT);
        break;
    case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(2000,
            agentState = (rmw_uros_ping_agent(500, 3) == RMW_RET_OK) ? AGENT_CONNECTED : AGENT_DISCONNECTED);
        if (agentState == AGENT_CONNECTED)
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
        break;
    case AGENT_DISCONNECTED:
        stopMotors();
        destroyEntities();
        agentState = WAITING_AGENT;
        Serial.println("[uROS] agent lost — reconnecting");
        break;
    }
    delay(1);
}
