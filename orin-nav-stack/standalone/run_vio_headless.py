# Headless D555 stereo-inertial (VIO) test for cuVSLAM on Orin.
#
# D555 differs from the USB D4xx the SDK's run_vio.py targets:
#   * IMU is a single COMBINED `motion` stream (not separate accel+gyro)
#   * ethernet/DDS device -> needs a DDS-enabled context
#   * firmware reports zero IMU noise/bias -> we supply values
#   * DDS driver reports identity cam<-imu extrinsics (no real rotation)
#
# cuVSLAM needs IMU + images fed on ONE monotonic timeline: all IMU up to an
# image's timestamp must be registered BEFORE that image is tracked, and never
# ahead of it. So the IMU thread only BUFFERS; the image loop drains the buffer
# up to each frame's timestamp, then tracks. (A free-running IMU thread races
# ahead and cuVSLAM rejects it as "non-monotonic".)
#
# No rerun; prints pose + drift-from-start so a stationary run objectively shows
# whether VIO is stable.  Keep the camera STILL for the test.
#
#   python3 run_vio_headless.py --seconds 35
import argparse
import collections
import threading
import time

import numpy as np
import pyrealsense2 as rs
import cuvslam as vslam
from scipy.spatial.transform import Rotation

from camera_utils import get_rs_camera
from run_stereo_headless import make_context

RES = (896, 504)
FPS = 30

# D555 firmware reports zero variances; start from the SDK's generic RealSense
# values (first tuning knob if VIO is jittery/divergent).
GYRO_NOISE_DENSITY = 6.0673370376614875e-03
GYRO_RANDOM_WALK = 3.6211951458325785e-05
ACCEL_NOISE_DENSITY = 3.3621979208052800e-02
ACCEL_RANDOM_WALK = 9.8256589971851467e-04

_stop = False
_imu_buf = collections.deque()      # (ts_ns, (ax,ay,az), (gx,gy,gz)) in arrival order
_buf_lock = threading.Lock()


def imu_pose_from_extrinsics(ext, use_rotation: bool) -> "vslam.Pose":
    t = np.array(ext.translation, dtype=float)
    if use_rotation:
        R = np.array(ext.rotation, dtype=float).reshape(3, 3).T  # rs2 is column-major
        q = Rotation.from_matrix(R).as_quat()
    else:
        q = Rotation.identity().as_quat()
    return vslam.Pose(rotation=q, translation=t)


def build_vio_rig(cp, use_imu_rotation: bool) -> "vslam.Rig":
    rig = vslam.Rig()
    rig.cameras = [
        get_rs_camera(cp['left']['intrinsics']),
        get_rs_camera(cp['right']['intrinsics'], cp['right']['extrinsics']),
    ]
    imu = vslam.ImuCalibration()
    imu.rig_from_imu = imu_pose_from_extrinsics(cp['imu']['cam_from_imu'], use_imu_rotation)
    imu.gyroscope_noise_density = GYRO_NOISE_DENSITY
    imu.gyroscope_random_walk = GYRO_RANDOM_WALK
    imu.accelerometer_noise_density = ACCEL_NOISE_DENSITY
    imu.accelerometer_random_walk = ACCEL_RANDOM_WALK
    imu.frequency = 200
    rig.imus = [imu]
    return rig


def imu_buffer_thread(ctx) -> None:
    """Read the D555 combined motion stream at 200 Hz and BUFFER it (no register)."""
    global _stop
    pipe = rs.pipeline(ctx)
    cfg = rs.config()
    cfg.enable_stream(rs.stream.motion)
    try:
        pipe.start(cfg)
    except Exception as e:
        print(f"[imu] could not start motion pipeline: {e}")
        return
    n = 0
    try:
        while not _stop:
            frames = pipe.wait_for_frames()
            ts = int(frames[0].timestamp * 1e6)
            cm = frames[0].as_motion_frame().get_combined_motion_data()
            a = (cm.linear_acceleration.x, cm.linear_acceleration.y, cm.linear_acceleration.z)
            g = (cm.angular_velocity.x, cm.angular_velocity.y, cm.angular_velocity.z)
            with _buf_lock:
                _imu_buf.append((ts, a, g))
            n += 1
    except Exception as e:
        print(f"[imu] thread error after {n} samples: {e}")
    finally:
        pipe.stop()
        print(f"[imu] buffered {n} samples total")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=int, default=35)
    ap.add_argument("--warmup", type=int, default=30)
    ap.add_argument("--identity-imu", action="store_true",
                    help="use identity IMU rotation instead of real extrinsics")
    args = ap.parse_args()

    ctx = make_context()

    # --- one-shot setup: stereo + motion, to read intrinsics/extrinsics ---
    sp = rs.pipeline(ctx)
    sc = rs.config()
    sc.enable_stream(rs.stream.infrared, 1, RES[0], RES[1], rs.format.y8, FPS)
    sc.enable_stream(rs.stream.infrared, 2, RES[0], RES[1], rs.format.y8, FPS)
    sc.enable_stream(rs.stream.motion)
    sp.start(sc)
    fr = sp.wait_for_frames()
    left = fr.get_infrared_frame(1)
    right = fr.get_infrared_frame(2)
    motion = next((f for f in fr if f.is_motion_frame()), None)
    cp = {'left': {}, 'right': {}, 'imu': {}}
    cp['left']['intrinsics'] = left.profile.as_video_stream_profile().intrinsics
    cp['right']['intrinsics'] = right.profile.as_video_stream_profile().intrinsics
    cp['right']['extrinsics'] = right.profile.get_extrinsics_to(left.profile)
    cp['imu']['cam_from_imu'] = motion.profile.get_extrinsics_to(left.profile)
    sp.stop()
    print(f"IMU rotation used: {'IDENTITY' if args.identity_imu else 'REAL extrinsics'}")

    cfg = vslam.Tracker.OdometryConfig(
        async_sba=False,
        enable_final_landmarks_export=True,
        enable_observations_export=True,
        debug_imu_mode=False,
        odometry_mode=vslam.Tracker.OdometryMode.Inertial,
        rectified_stereo_camera=True,
    )
    rig = build_vio_rig(cp, use_imu_rotation=not args.identity_imu)
    tracker = vslam.Tracker(rig, cfg)
    print("[ok] Inertial (VIO) tracker constructed on this device")

    global _stop
    th = threading.Thread(target=imu_buffer_thread, args=(ctx,), daemon=True)
    th.start()

    ip = rs.pipeline(ctx)
    ic = rs.config()
    ic.enable_stream(rs.stream.infrared, 1, RES[0], RES[1], rs.format.y8, FPS)
    ic.enable_stream(rs.stream.infrared, 2, RES[0], RES[1], rs.format.y8, FPS)
    prof = ip.start(ic)
    ds = prof.get_device().query_sensors()[0]
    if ds.supports(rs.option.emitter_enabled):
        ds.set_option(rs.option.emitter_enabled, 0)

    m = vslam.ImuMeasurement()
    t0 = time.time()
    n = 0
    valid = 0
    first_pos = None
    max_drift = 0.0
    t_last = t0
    try:
        while time.time() - t0 < args.seconds:
            f = ip.wait_for_frames()
            lf = f.get_infrared_frame(1)
            rf = f.get_infrared_frame(2)
            if not lf or not rf:
                continue
            t_img = int(lf.timestamp * 1e6)

            # Register all buffered IMU with ts <= this image's ts, in order.
            while True:
                with _buf_lock:
                    if not _imu_buf or _imu_buf[0][0] > t_img:
                        break
                    ts_i, a, g = _imu_buf.popleft()
                m.timestamp_ns = ts_i
                m.linear_accelerations = np.asarray(a, dtype=np.float32)
                m.angular_velocities = np.asarray(g, dtype=np.float32)
                tracker.register_imu_measurement(0, m)

            images = (np.asanyarray(lf.get_data()), np.asanyarray(rf.get_data()))
            est, _ = tracker.track(t_img, images)
            n += 1
            if est.world_from_rig is None:
                if n > args.warmup:
                    print("[warn] pose invalid")
                continue
            if n <= args.warmup:
                continue
            valid += 1
            pos = np.array(est.world_from_rig.pose.translation)
            if first_pos is None:
                first_pos = pos.copy()
            drift = float(np.linalg.norm(pos - first_pos))
            max_drift = max(max_drift, drift)
            if valid % 30 == 0:
                now = time.time()
                fps = 30.0 / (now - t_last) if now > t_last else 0.0
                t_last = now
                print(f"[vio] valid={valid} fps~{fps:4.1f} "
                      f"pos=({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f}) "
                      f"drift_from_start={drift:.3f} m")
    except KeyboardInterrupt:
        pass
    finally:
        _stop = True
        ip.stop()
        time.sleep(0.4)
    verdict = "STABLE" if max_drift < 0.30 else ("DRIFTING" if max_drift < 5 else "DIVERGED")
    print(f"[done] valid={valid} max_drift_from_start={max_drift:.3f} m over {args.seconds}s "
          f"(stationary target < ~0.3 m)  => {verdict}")


if __name__ == "__main__":
    main()
