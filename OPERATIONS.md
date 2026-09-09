# Operations — the runbook

How to bring the rover up, measure it, calibrate it, and diagnose it when it
misbehaves. For *why* anything is the way it is see
[ARCHITECTURE.md](ARCHITECTURE.md); for the measured numbers see
[PHASE1.md](PHASE1.md).

---

## 1. Bring-up

> Coming back from a power cycle? **[STARTUP.md](STARTUP.md)** is the shorter
> path: it covers all five boxes, not just the Jetson stack, and ends with the
> Studio link. This section is the reference for what each layer proves.

Run these **in order**. Each checks the layer beneath it, so a failure names its
own layer instead of hiding in a wall of log.

```bash
./rover camera      # the D555 alone
./rover pose        # + cuVSLAM and the gyro
./rover fused       # + the fused pose -> /odom and TF
./rover map         # + nvblox: build the room as it drives
./rover nav         # + nav2: plan a route and drive it
./rover vlm         # + the VLM bridge: a picked pixel -> a nav2 goal (phase 4)
```

| command | what it proves |
|---|---|
| `./rover camera` | streaming ≥15 Hz **and** the IR emitter verified OFF by read-back |
| `./rover pose` | cuVSLAM ≥10 Hz, gyro ≥50 Hz, re-framed into `base_link` |
| `./rover fused` | `/odom` ≥15 Hz, TF `odom → base_link` owned by exactly one node |
| `./rover map` | the occupancy grid publishing, and how much is mapped |
| `./rover nav` | both costmaps carrying obstacle data, all servers activated |
| `./rover vlm` | compressed colour ≥2 Hz, and `pixel_to_goal` subscribed to `/vision/pixel_query` |

**Restarting `fused` destroys the map.** The pose origin resets, so the old
geometry would land in the wrong place. Restart `map` whenever you restart
`fused`.

Then, at any time:

```bash
./rover status      # every layer's rate, one screen
./rover values      # one-shot readout of every sensor
./rover wheels      # the ESP32 link specifically
./rover logs vo     # tail a layer's log: camera | vo | gyro | fused
./rover stop        # tear the container down
```

### Before you trust anything

**Rates are the honest signal.** Every failure this rig has had was a rate
collapsing, not a topic disappearing. `./rover status` is the first thing to
look at when something is odd.

---

## 2. Driving it

Teleop lives on the Pi 5: **`http://192.168.1.16:8091`** from a phone on the same
WiFi.

**It defaults to AUTO and the buttons do nothing until you flip it to MANUAL.**
That is nav2/the brain owning `/cmd_vel`, not a fault. In MANUAL it publishes at
10 Hz whether or not a button is held, so `/cmd_vel` traffic alone proves
nothing — only a non-zero velocity does.

| button | commanded | speed at the camera | |
|---|---|---|---|
| forward / back | 0.20 m/s | 20 cm/s | ✅ under the tracking limit |
| left / right pivot | 2.0 rad/s | **34 cm/s** | ⚠ over it — expect jumps |

Release is a **coast**, not a brake: the wheels decay to zero over ~0.5 s. Motion
is gated on a `/cmd_vel` newer than 500 ms, so a lost connection stops the rover.

---

## 3. The gates

```bash
./rover compare --expect 2.00    # push 2.00 m straight  -> want 190–210 cm
./rover compare --return         # out and back          -> want ≤ 10 cm
./rover compare --spin 360       # rotate 360°           -> want ≤ 10°
./rover compare                  # no gate: just watch every sensor live
```

**Procedure, every time:**

1. **Keep still for the first 5 seconds** — the gyro bias is being measured. Any
   nudge poisons it.
2. Mark where the rover is.
3. Drive or push.
4. **Ctrl-C** to grade and write the CSV.

**Keep clutter in view.** A bare wall starves the tracker — landmarks below 30
and cuVSLAM is dropped from the fusion. Furniture, doorways, chair legs are what
it needs.

### Reading the output

```
  source        x cm     y cm    th deg   straight cm   path cm      Hz
  cuvslam       -0.1      0.0     -0.00          0.1       0.0    30.0
  wheels         0.0      0.0      0.00          0.0       0.0    20.0
  gyro             —        —      0.07            —         —   200.9
  FUSED         -0.1      0.0      0.03          0.1       0.0    29.0

  health: cuvslam OK  wheels OK  gyro OK   |  all three contributing
```

| column | meaning |
|---|---|
| `x`, `y` | displacement from the start, cm |
| `th` | heading change since the start, degrees |
| `straight` | straight-line distance from the start — **what a tape measures** |
| `path` | total distance travelled — larger if it wandered or reversed |
| `Hz` | publish rate |

`FUSED` is the row to navigate on. The others are how you know it is honest.

Also watch:

- **`landmarks`** — under 30 and cuVSLAM is dropped from the fusion
- **`health:`** — who is covering for whom, right now
- **`STILL — retuning bias`** — the gyro bias tracker working
- **`encoder scale`** — the measured wheels/cuVSLAM ratio, and whether it clamped
- **`tilt:`** — roll, pitch, and the camera's measured mount pitch

### Logs

Every run writes a timestamped CSV to `logs/`, on the host, surviving container
teardown:

```bash
ls -lt logs/compare-*.csv | head
```

Per-source `x, y, th_deg, straight, path, hz`, plus the raw values: `roll_deg`,
`pitch_deg`, `landmarks`, `accel_x/y/z`, `gyro_x/y/z`, `velL`, `velR`, and all
four cumulative tick counts.

---

## 4. Calibration

All of these run inside the container. Prefix with:

```bash
docker exec -it rover bash -lc 'source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=0; <command>'
```

| what | command | procedure |
|---|---|---|
| **encoder m/count** | `python3 -u /logs/calibrate_encoders.py 200` | push straight forward exactly 200 cm, forward only |
| **skid-steer track width** | `python3 -u /logs/calibrate_rotation.py 360` | align to a floor line, rotate one full turn back to it |
| **all four encoders alive** | `python3 -u /logs/wheels_selfpaced.py 60` | rover lifted, spin each wheel in any order |
| **yaw sign (REP-103)** | `python3 -u /logs/check_yaw_sign.py 25` | hold the LEFT button; expects a positive rate |
| **all rates + board health** | `python3 -u /logs/check_rates.py` | nothing — it just listens |
| **teleop end to end** | `python3 -u /logs/watch_teleop.py 25` | press FORWARD during the window |
| **fusion equivalence** | `python3 -u /logs/check_equivalence.py` | run beside `compare.py`, drive, compare endpoints |
| **does the driven line close?** | `python3 -u /logs/loop_test.py` | drive a loop back to the start, graded on `/odom` |
| **how much is mapped** | `python3 -u /logs/map_stats.py` | nothing — it just reads |
| **the map, as text** | `python3 -u /logs/map_view.py` | nothing; `--once` for a single frame |
| **why won't it pivot?** | `python3 -u /logs/turn_diag.py 40` | hold LEFT then RIGHT during the window |

### Calibration rules learned the hard way

- **Calibrate against a tape, never against another sensor.** cuVSLAM reads ~2%
  under; calibrating to it bakes that into every wheel.
- **Forward only.** Reversing lets a counting error cancel itself and hide.
- **A 360° beats a 90°.** Four times the signal, and returning to the same floor
  line is far easier to judge than eyeballing a right angle.
- **Never compare cumulative counts (arc length) against `straight`
  (displacement).** On a path that curves or doubles back they are different
  quantities. This produced a confident "the encoders are 2× out" that was
  entirely wrong.

---

## 5. Troubleshooting, by symptom

### The camera

**Three distinct failures that look alike.** Tell them apart before acting:

| symptom | what is true | fix |
|---|---|---|
| `No RealSense devices` **and** `ethtool` says `Link detected: no` | no electrical link — cable or PoE injector | reseat the cable; nothing on the camera can help |
| `dds-device.cpp:46 device is offline`, driver alive, 0 publishers | link fine, on-camera DDS server dead | unplug 5 s, replug |
| streams at 30 Hz but `ros2 param set` times out | **half-alive**: image traffic works, option traffic does not | power-cycle, then **wait 30 s** before re-attaching |

```bash
cat /sys/class/net/enP8p1s0/carrier     # 1 = cable link is up
ping -c2 192.168.11.55                  # answers even when completely dead
```

**Ping is not a health check. Nor is a live image stream.** The only proof is the
emitter read-back, which `./rover camera` does for you and refuses to continue
without.

### The wheels

```bash
./rover wheels                          # rate at the Jetson
ros2 topic echo /rover_diag --once      # the board's own report
```

`/rover_diag` gives `x` = `loop()` Hz, `y` = free heap KB, `z` = agent state
(2 = connected). **If `loop()` is in the hundreds, the firmware is fine and the
problem is the network.** That distinction used to cost a day.

Jitter with a healthy `loop()` means WiFi. Check signal:

```bash
iw dev wlP1p1s0 link | grep -iE 'signal|bitrate'
```

−36 dBm is good; −47 dBm and rising jitter means the rover has moved away from
the access point.

### The pose

| symptom | likely cause |
|---|---|
| `straight` grows while parked | gyro bias stale — should self-correct; check `STILL — retuning bias` appears |
| many teleports, `landmarks` low | featureless scene. Drive where there is furniture, not a bare wall |
| many teleports, `landmarks` healthy | genuinely unexplained. Speed is NOT the answer — 89 cm/s with 0 teleports on 2026-08-22. See TODO §3 |
| `wheels` row wildly wrong in turns | expected — skid-steer scrub. The gyro owns heading |
| `FUSED` worse than an input | a real bug. This has happened twice; see PHASE1.md §7 |

### The map

| symptom | likely cause |
|---|---|
| RViz map display empty, topic alive | **QoS.** nvblox is VOLATILE, nav2's costmaps are TRANSIENT_LOCAL. RViz renders a mismatch exactly like a dead publisher |
| subscriber gets nothing, `topic hz` shows 5 Hz | wrong topic: `static_map_slice` is a `DistanceMapSlice`. The `OccupancyGrid` is `static_occupancy_grid` |
| `./rover map` gate reads 0 Hz | checked too early — nvblox allocates GPU hash tables before publishing. The gate waits 25 s |
| walls doubled or offset | the pose drifted; the map was written twice in different places |
| map smears suddenly | a cuVSLAM teleport. **Unrecoverable** — nvblox cannot un-write it. Check `jumps` in `/fusion/status` and restart the map |
| large areas blank despite driving there | the camera never saw them — a wall behind the rover stays unknown until it turns |

### nav2

| symptom | likely cause |
|---|---|
| `bt_navigator` fails: *"spin action server not available"* | a behaviour tree still references Spin. **Both** trees need overriding, `navigate_to_pose` and `navigate_through_poses` |
| costmap topic silent but the node is fine | `always_send_full_costmap: false` — it only sends deltas after the first full map |
| `Invalid frame ID "odom"` at startup | a race; the costmaps come up before the TF is flowing and recover |
| rover sits still after a goal | it was told to rotate in place, which it cannot do. See TODO §14 and the `NO-PIVOT` settings |
| goal rejected as unreachable | the GOAL cell is blocked, or the rover is standing in one. `logs/around.py` shows which; `logs/where.py` lists what IS reachable |

### RViz shows nothing

**Check this first: was it started with `-d`?**

```bash
bash ~/rover_live.sh          # correct -- on the laptop, in a desktop terminal
rviz2 -d ~/rover_live.rviz    # correct, the long way
rviz2 ~/rover_live.rviz       # WRONG -- silently ignored
```

`rviz2` ignores a bare config path. Without `-d` it starts with its **defaults**:
Fixed Frame `map`, which does not exist on this rover, and **zero displays**. A
blank window, no error, and the ROS graph shows `/rviz` connected while
subscribing to nothing. Use `./rover view` from the Jetson, or `bash
~/rover_live.sh` on the laptop — neither can get this wrong.

**`./rover view` needs somebody logged in at the laptop desktop.** RViz started
over ssh into a login screen renders into a void and exits 0. The launcher
refuses in that case and says so rather than reporting success; log in there and
re-run, or start it on the laptop by hand with the command above.

| symptom | cause |
|---|---|
| blank, `/rviz` in the graph, **0 subscribers** on `/odom` | started without `-d` |
| blank, displays listed, Global Status red | Fixed Frame is `map`; set it to `odom` |
| one display blank, others fine | **QoS.** nvblox is Volatile, nav2 costmaps are Transient Local |
| `GLSL link result: active samplers...` | a driver quirk in RViz's Map shader; usually still draws |

To confirm from the rover whether RViz is really subscribing:

```bash
ros2 topic info /odom        # Subscription count should be >= 1
```

### Teleop

| symptom | cause |
|---|---|
| buttons do nothing | still in **AUTO** — flip to MANUAL |
| turn command drives forward/backward | not enough duty to scrub four tyres. See TODO §14 — `WZ` must be ~5.0 for this firmware, not 2.0 |
| `/cmd_vel` traffic but nothing moves | in MANUAL it publishes 10 Hz of zeros whether or not a button is held. Only a **non-zero** velocity means anything |

Restart it **without a password** — it runs as your user with `Restart=always`:

```bash
ssh 192.168.1.16 'pkill -f teleop_web.py'
```

### Everything looks alive but nodes cannot see each other

A stale `ROS_DISCOVERY_SERVER`. `./rover` unsets it on every command; if you are
running something by hand, do the same.

---

## 6. Flashing the ESP32

Full instructions in [phase1/firmware/FLASHING.md](phase1/firmware/FLASHING.md).
In short:

1. Pull the repo on a machine with the Arduino IDE.
2. **Set `WIFI_PASS` on line 68 — locally only. Do not commit it back.**
3. Board: ESP32 Dev Module. Requires **esp32 core 3.x** (the firmware uses the
   3.x per-pin PWM API) and **micro_ros_arduino, jazzy branch**.
4. Watch the serial monitor at 115200 for `[uROS] entities live`.

If you see `[uROS] FAILED to create <topic> — entity limit?`, the precompiled
library's entity caps have been exceeded. Drop `/rover_diag` first, then
`/wheel_odom`. **Never drop `/wheel_ticks`** — it is the distance reference.

---

## 7. The brain — LangGraph Studio on the Pi 5

The agent graph the Pi 5 runs can be opened in LangGraph Studio: every node,
every tool call and every handover, live. Useful for reading the agentic flow
rather than inferring it from Telegram replies.

### It is not started by anything

Nothing on the Pi 5 starts it at boot — `agent_node` runs the graph in-process
and needs no server. Studio is a **separate dev server** you start by hand.

```bash
ssh 192.168.1.16 'setsid nohup ~/ros2_ws/scripts/start_studio.sh \
    > /tmp/langgraph_dev.log 2>&1 < /dev/null &'
```

`~/ros2_ws/scripts/start_studio.sh` on the Pi 5 holds the environment, and holds it for
a reason — every line in it is something that fails quietly if you get it wrong:

- **It `cd`s to `~/ros2_ws` itself.** The CLI reads `langgraph.json` and `.env`
  from the working directory, not from its own location.
- **Full path to `~/.local/bin/langgraph`.** It is not on a non-interactive ssh
  PATH, so `ssh pi 'langgraph dev'` is `command not found`.
- **ROS vars matched to `agent_node`'s** — plain SUBNET discovery, domain 0. A
  stale `ROS_DISCOVERY_SERVER` gives a bridge that starts perfectly cleanly and
  silently sees no robot.
- **`LANGROBO_MEMORY_PATH` set to a Studio-only store**, and deliberately NOT in
  `.env` — `agent_node` reads that same file, so putting it there would repoint
  the *robot's* memory. See the episodic-memory note below.
- **No `set -u`.** ROS's `setup.bash` references unset variables, so `set -u`
  aborts the script the instant it is sourced — silently, if its stderr is going
  to `/dev/null`.
- **`--allow-blocking`**, because the graph does synchronous ROS and HTTP work
  and the server otherwise raises on the first blocking call.

It takes 30-60 s to answer. Poll rather than assume:

```bash
ssh 192.168.1.16 'curl -s http://127.0.0.1:2024/ok'     # {"ok":true} when ready
```

Log: `/tmp/langgraph_dev.log`. To stop it:

```bash
ssh 192.168.1.16 'pkill -f "langgrap[h] dev"'
```

**The brackets are not a typo and not optional.** `pkill -f "langgraph dev"`
matches its OWN command line — the pattern text is *in* the command sshd is
running — so the remote shell kills itself and its own ssh session. You get
exit 255, no output, and nothing killed, which reads exactly like the Pi 5's
intermittent ssh. `langgrap[h]` is a regex that does not match the literal
string `langgrap[h]`, so it only matches the server.

### Reaching the UI

It binds to localhost deliberately. Tunnel from the machine with the browser:

```bash
ssh -N -L 2024:localhost:2024 rakhi24@192.168.1.16
```

then open `https://smith.langchain.com/studio/?baseUrl=http://127.0.0.1:2024`.

**The tunnel is not just convenience.** Studio is served over HTTPS, so a
browser blocks it calling `http://192.168.1.16:2024` as mixed content.
`http://127.0.0.1` is exempt as a secure context, so tunnelling is what makes
the hosted UI work at all. `langgraph dev --tunnel` is the alternative and puts
a public HTTPS URL in front of the robot's agent for the life of the process.

Health checks, through the same tunnel:

```bash
curl localhost:2024/ok       # {"ok":true}
curl localhost:2024/info     # versions and feature flags
curl -XPOST localhost:2024/assistants/search \
     -H 'Content-Type: application/json' -d '{}'
```

The graph is `agent`, from `graph_studio.py:graph` via `langgraph.json`.

### Three things that surprise you

- **A `navigate` turn in the browser drives the real rover.**
  `graph_studio.py` attaches a real `ROS2Bridge` whenever ROS is sourced, and
  `ros2 node list` then shows `/studio_bridge` alongside `/agent_node` — two
  bridges that can both publish `/cmd_vel`. Everything in §8 applies: a human
  watches, and MANUAL on the phone is the stop. To read flows **without** that
  risk, start it in a shell where ROS is *not* sourced: it falls back to
  `StubBridge` and tool calls are logged instead of published.
- **Studio has its own episodic memory, separate from the robot's.** Embedded
  Qdrant is single-process, so while `agent_node` holds `~/.langrobo/qdrant`
  Studio used to start with no memory at all (`Storage folder … is already
  accessed by another instance`). `start_studio.sh` now points it at
  `~/.langrobo/qdrant_studio`, so the memory tools work — but they recall
  **Studio's** memories, not the household's. That is the right trade for a dev
  tool and it cannot corrupt the robot's store. To genuinely SHARE one store,
  run a Qdrant server and set `QDRANT_URL` for both processes (`memory.py`
  supports it); that needs `agent_node` restarted and is not done.
- **`watchfiles` logs "N changes detected" every ~10 s and nothing is
  reloading.** `langgraph dev` writes its own `.langgraph_api/*.pckl`
  checkpoints inside the directory it watches. Confirm a real reload by looking
  for a second `ROS2 bridge active` in the log — one occurrence means it has
  imported the graph once.

### LangSmith, which needs none of this

`.env` sets `LANGCHAIN_TRACING_V2=true` and `LANGCHAIN_PROJECT=pi5`, so real
turns through `agent_node` are **already** traced to smith.langchain.com with no
server running and nothing tunnelled. For reading past flows that is the lower
effort path; Studio is for driving the graph yourself.

`STUDIO_MODEL` used to read `gpt-4o-mini` while `STUDIO_PROVIDER=llamacpp`,
which made every trace look like it had called OpenAI. It is now `default`,
matching `agent_params.yaml` — the honest llama.cpp convention, since the server
serves whatever model it has loaded (currently `gemma-4-12B-it-Q4_K_M`) and
ignores the field.

---

## 8. Safety

- **Hand-pushing is always safe.** `pidStep()` returns 0 below 0.01 m/s and
  `driveSide()` with zero duty coasts rather than brakes, so the wheels
  free-wheel and the PID will not fight you.
- **Motion is double-gated**: on `controlEnabled` (agent connected) *and* a
  `/cmd_vel` newer than 500 ms. An idle rover cannot creep.
- **Lift the rover onto blocks** for any wheel test. It is safer than clear floor
  and the wheels spin freely.
- **Never run `logs/keepalive.py` while teleoperating.** It publishes zero
  `/cmd_vel` continuously, which would interleave with real commands. It exists
  only as a fallback if a future flash regresses the executor fix.
