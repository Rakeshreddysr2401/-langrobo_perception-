# Fleet check — 2026-09-06

Answers one question: is every box alive, can I talk to the robot, and if I
ask it to go somewhere does it actually move. Measured live on branch
`rover-v1.0.7`, full stack already up (container `rover`, 56 min uptime).
Nothing simulated, nothing inferred from docs.

---

## The five boxes

| box | address | state |
|---|---|---|
| Jetson Orin Nano | 192.168.1.15 | **OK** — container `rover` up, all layers running |
| Pi 5 | 192.168.1.16 | **OK** — teleop `:8091` in AUTO, `agent_node` brain up, 6.3 GiB free |
| Mac mini | **192.168.1.6** (`singireddys-mac-mini.local`) | **OK — and it is not idle**, see below |
| D555 depth camera | 192.168.11.55 (PoE) | **OK, one rate low**, see below |
| ESP32 wheel link | via micro-ROS | **OK** — `/wheel_state` 21.2 Hz, `/cmd_vel` subs 1 |
| Laptop (RViz) | DHCP, .10 answered ARP | not verified this pass |

### The Mac mini is the brain's LLM, and it is live

`PLAN.md:19` still says "mac mini (vlm currently not active)". That is stale.
The Pi 5's `agent_node` reports:

```
"llm":{"provider":"llamacpp",
       "base_url":"http://singireddys-mac-mini.local:8080/v1",
       "primary_available":true, "fallback":null}
```

It serves `gemma-4-12B-it-Q4_K_M.gguf`, capabilities `["completion","multimodal"]`
— so it is the VLM too. `/v1/models` answers 200 from **both** the Jetson and
the Pi 5, so it is not a one-machine-only route. `PLAN.md` needs updating.

---

## Rates, measured (`./rover status`)

```
camera  /camera/camera0/infra1/image_rect_raw   25.8   want 15   OK
camera  /camera/camera0/depth/image_rect_raw     8.3   want 10   LOW  <-- only amber
pose    /vo/odom                                25.3   want 10   OK
pose    /gyro/base                             199.1   want 50   OK
wheels  /wheel_state                            21.2   want 15   OK
fused   /odom                                   20.0   want 15   OK
map     /nvblox_node/static_occupancy_grid       4.8   want  1   OK
nav     /global_costmap/costmap                  0.8   want0.5   OK
nav     /local_costmap/costmap                   1.7   want  1   OK
vlm     /camera/color/image_raw/compressed       4.5   want  2   OK
view    /rover/model                             1.0   want0.5   OK
```

**Depth is the one number under its gate** — 8.3 Hz against a want of 10. It is
not currently breaking anything downstream (nvblox is at 4.8 Hz, both costmaps
are green), but it is the stream `pixel_to_goal.py` reads to turn a VLM pixel
into a metric goal, so it is worth watching rather than ignoring. Plausible
cause: colour was enabled on this camera 2026-09-06 for the VLM path and
`align_depth` now makes the driver do a reprojection it did not do before.

### cuVSLAM is genuinely tracking, not silently dead-reckoning

This is the check that matters and that the layer gates do not cover (TODO §21
— it has diverged three times and the fallback is silent):

```
cuvslam  vo_z -0.014 m   implausible 0   dead-reckoned 0.00 m   landmarks 61
         ok — tracking, not dead reckoning
```

---

## Can I talk to it?

**Text: yes, right now.** Telegram is configured, polling, 1 member, no last
error. `turns_total: 2`, last turn 2.07 s. That channel is live.

**Voice: no, not right now.** `pi5_stt_node` / `pi5_tts_node` from
`pi5_voice_pkg` are **not running** — the Pi 5 has `teleop_web.py` and
`agent_node` (via `brain_launch.py provider:=llamacpp start_micro_ros:=false
robot_body:=rover`) and nothing else. The package is built and documented
(`~/ros2_ws/PI5_VOICE.md` on the Pi 5), so this is "not started", not "not
working" — but it has to be started before anyone can speak to the robot.

Note also, from that doc: **Bluetooth pairing for the boAt Stone mic does not
survive a reboot** (`Paired: no` even though `Trusted: yes`), so the speaker
needs a manual reconnect before voice will hear anything.

The Jetson cannot host voice while driving — that is settled and measured, see
`VOICE_PLACEMENT.md`. Voice belongs on the Pi 5.

---

## If I ask it to go somewhere, does it move?

The whole chain is wired and every link was verified individually this pass:

```
speech/Telegram -> agent_node (Pi 5)
                -> navigate_to_visible_object() / navigate_to_pose() / move_robot()
                -> /vision/pixel_query      ---> pixel_to_goal.py (Jetson)
                -> NavigateToPose goal, frame odom
                -> nav2 -> /cmd_vel -> ESP32 -> wheels
```

Verified:

- The Pi 5 can see the Jetson's nav2 across DDS: `ros2 action list` on the Pi 5
  returns `/navigate_to_pose`, `/compute_path_to_pose`, `/follow_path`, `/backup`.
- `/vision/pixel_query`, `/vision/pixel_result`, `/goal_pose`, `/cmd_vel` are all
  visible from the Pi 5.
- `image_bridge` and `pixel_to_goal` (the phase-4 nodes) are both running in the
  Jetson container, and `agent_node` reports `camera_frame_age_s: 0.03` — the
  brain is receiving frames.
- **The `frame_id` bug is already fixed.** `pixel_to_goal.py`'s docstring warns
  that `_nav_worker` hardcoded `"map"` on the Pi 5 and that `start_nav_to_pose`
  could not work on this rig until that became `"odom"`. It has been:
  `langrobo_ros/ros2_bridge.py:597` reads `goal.pose.header.frame_id = "odom"`.
- Teleop is in **AUTO** (`GET :8091/mode` -> `{"manual": false}`), so it is not
  streaming the zeros that cancel nav2's commands.
- Agent movement tools present: `move_robot`, `navigate_to_pose`,
  `navigate_to_visible_object`, `save_location`, `point_camera`.

**Not verified: an actual commanded drive.** No motion was commanded during this
check — the rig's standing rule is that a human watches every autonomous move,
and nobody was standing over it. So "the plumbing is proven live" is the honest
claim; "it drove" is not.

One caveat specific to *"go near the wall"*: that phrasing routes to
`navigate_to_visible_object`, which asks the VLM to pick a **pixel on an
object**. A wall is a poor object for that — it is not a bounded thing, and
whichever pixel comes back decides the goal. `move_robot("forward")` and
`navigate_to_pose(<saved location>)` are the better-conditioned paths. Worth an
actual test rather than an assumption.

---

## Open items from this pass

1. **Depth at 8.3 Hz**, under its 10 Hz gate. Watch; likely the new `align_depth`.
2. **Voice not started** on the Pi 5, and the BT mic needs a post-reboot reconnect.
3. **`PLAN.md:19` is stale** — the Mac mini VLM/LLM is active at 192.168.1.6.
4. **SSH to the Pi 5 went flaky** mid-check: the first several sessions worked,
   then every session hung despite ping succeeding and port 22 accepting the
   TCP connection. The Jetson reaches it over Wi-Fi (`wlP1p1s0`) while that link
   also carries DDS, so contention is the first suspect. Not diagnosed.
5. **`navigate_to_visible_object("wall")`** untested end-to-end — see above.

## Standing safety note

Unchanged and worth repeating: the rover is blind below 10 cm, above 24 cm,
outside 87°, and **downward** — there is no drop-off detection at all. Tapping
MANUAL on the phone cancels the active nav2 goal; that is the stop.
