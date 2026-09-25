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

---

# Voice bring-up attempt — 2026-09-06, same session

Asked to start the Pi 5 voice stack and reconnect the mic. **Blocked, and the
blocker is physical, not software.** Everything on the software side checked
out; there is simply no audio hardware attached to the Pi 5 right now.

## What passed

- **No conflict with the Jetson.** `fleet_role.sh voice status` -> `jetson
  voice: stopped`, and no whisper/kokoro/ai_stack processes on the Jetson.
  The doc's "never run this alongside the Jetson's ai_stack voice role"
  precondition is satisfied.
- **API keys present** in `~/ros2_ws/.env` on the Pi 5: `SARVAM_API_KEY`,
  `SONIOX_API_KEY` (plus OPENAI/LANGSMITH/LANGCHAIN/TAVILY).
- **Config read** (`pi5_voice_pkg/config/voice_params.yaml`):
  `stt_provider: sarvam`, `tts_provider: sarvam_translate`,
  `wake_detector: openwakeword`, `wake_word: hey_jarvis`, `require_wake: true`,
  `bt_mac: D6:AA:BB:59:EF:B6`, `bt_profile: hfp`.

## What blocked it

**The boAt Stone 650 is not reachable.**

```
bluetoothctl info D6:AA:BB:59:EF:B6
    Paired: no      Bonded: no      Trusted: yes      Connected: no

bluetoothctl connect D6:AA:BB:59:EF:B6
    Failed to connect: org.bluez.Error.Failed br-connection-page-timeout
```

`br-connection-page-timeout` means the speaker never answered the connection
page — it is powered off, out of range, or already connected to another device
(a phone will hold it). An 18 s inquiry scan did **not** see it: it appears in
`bluetoothctl devices` only as a remembered pairing, not as a live result.

**And there is no fallback mic.** The documented wired-headset fallback path is
not plugged in either:

```
arecord -l          ->  (no capture hardware devices at all)
pactl list short sources  ->  auto_null.monitor   [dummy]
pactl list short sinks    ->  auto_null           [dummy]
```

`auto_null` is PipeWire's placeholder sink, which is what you get when no real
output device exists. So at this moment the Pi 5 has **neither a microphone nor
a speaker**. Starting `voice_launch.py` now would bring both nodes up against
no device — `bt_audio.ensure()` never raises by design, so they would start and
then be unable to open an input stream. Not started, deliberately.

## To unblock (physical, ~1 minute)

1. Power on the boAt Stone 650, and if a phone is holding it, disconnect it
   there first — a Bluetooth speaker will only page-answer for one host.
2. Then, on the Pi 5:
   ```bash
   bluetoothctl connect D6:AA:BB:59:EF:B6
   bluetoothctl info D6:AA:BB:59:EF:B6 | grep -E 'Paired|Connected'
   ```
   Want `Paired: yes`, `Connected: yes`. `bt_audio.ensure()` handles the
   `hfp` profile switch and the PipeWire default-sink/source wiring itself.
3. Then the launch, which is one command:
   ```bash
   cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash
   ros2 launch pi5_voice_pkg voice_launch.py
   ```
4. Say **"hey jarvis"** first — `require_wake: true`, so nothing is transcribed
   until the wake word fires.

A wired USB headset in the Pi 5 also works and skips steps 1-2 entirely.

## Two things to know before the first utterance

- **It will answer in Telugu.** `tts_provider: sarvam_translate` translates the
  English reply text to Telugu speech. That is the verified-working config from
  2026-09-04, not a misconfiguration — but set `tts_provider: local` (Kokoro)
  for English if that is not what is wanted right now.
- **The first turn may take 50-108 s, and that is not a fault.** Documented
  cold-start behaviour (KV cache cold, or a stale HTTP connection to the Mac
  mini). Warm turns are ~1.5 s. Ask a second time before concluding voice is
  broken; barge-in correctly abandons the stuck turn.

## Still true

Telegram remains the working channel today and needs none of the above.

---

# Post-power-cycle bring-up — 2026-09-09

Everything was power-cycled (D555, Jetson, Pi 5, ESP32) and the Pi 5 refactored.
Cold start from `docker start rover`, every layer brought up in order.

## All eleven rows green

```
camera  /camera/camera0/infra1/image_rect_raw   29.9   want 15    OK
camera  /camera/camera0/depth/image_rect_raw    23.0   want 10    OK
pose    /vo/odom                                28.5   want 10    OK
pose    /gyro/base                             200.1   want 50    OK
wheels  /wheel_state                            19.9   want 15    OK
fused   /odom                                   20.0   want 15    OK
map     /nvblox_node/static_occupancy_grid       4.9   want  1    OK
nav     /global_costmap/costmap                  0.8   want0.5    OK
nav     /local_costmap/costmap                   1.7   want  1    OK
vlm     /camera/color/image_raw/compressed       4.6   want  2    OK
view    /rover/model                             1.0   want0.5    OK

cuvslam  vo_z +0.003 m  implausible 0  dead-reckoned 0.00 m  landmarks 215
         ok — tracking, not dead reckoning
```

**The ESP32 came back on its own** — `/wheel_state` 19.9 Hz, `/cmd_vel` subs 1,
no power-cycle needed. That is not the §24 regression state. Teleop is in
**AUTO** (`GET :8091/mode` -> `{"manual": false}`).

Note `./rover status` with nothing running does not fail fast — it walks eleven
topics at ~12 s each and takes over two minutes to print a table of dashes. Run
a layer first; the layer gates are the fast answer.

## The 8.3 Hz depth reading was misattributed

Open item 1 from 2026-09-06 blamed `align_depth` for depth sitting under its
gate. **That hypothesis is wrong.** The camera launch args are byte-identical
(`enable_color:=true`, `align_depth.enable:=true` — `rover:104`), and measured
across this bring-up:

| when | depth |
|---|---|
| camera/pose/fused/map/nav up, **no VLM layer** | 23–26 Hz |
| after `./rover vlm` attaches `image_bridge` | **14.3 Hz**, max gap 633 ms |

So it is **consumer load**, not the driver's reprojection. `align_depth` was
already on in both readings. The 8.3 Hz on 2026-09-06 was the VLM layer running.

Still above the 10 Hz gate, so nothing is broken — but the 633 ms worst-case
inter-frame gap is the number to watch, not the average. Depth is what
`pixel_to_goal.py` reads to turn a VLM pixel into a metric goal, and a 0.6 s
stall there lands the goal wherever the rover was two thirds of a second ago.

## `./rover vlm` is not in the documented bring-up order

The `rover-start` skill lists camera -> pose -> fused -> map -> nav -> view and
stops. `vlm` is a real layer with its own gate and is not in that list, so a
by-the-book bring-up leaves the VLM row at `--` and the Pi 5 brain's
`look()` / `approach_described_object()` path silently dead. It is not a bug in
the stack — it is a gap in the written order. Start it after `nav`.

**Fixed 2026-09-09**: the `rover-start` skill now lists `vlm` in the bring-up
order, with the depth cost noted. The same pass corrected two other stale
instructions in it — it still claimed `./rover fused`'s gate does not cover
cuVSLAM (it does now), and it recommended checking divergence with
`ros2 topic echo /fusion/status --once`, which `health.py` documents as
truncating that JSON, so anything grepping it reads nothing.

## Fixed this session

- The cuVSLAM honesty check moved from a doc instruction into the gates — see
  TODO §21, "Surface it". `./rover fused` now runs it and warns; `./rover map`
  now refuses to build on a dishonest pose.
- **`compare.py` now shows divergence** (TODO §21, the last open bullet). It was
  watching `/vo/status`, which does not carry `vo_z` / `vo_implausible` /
  `dr_metres` — those are on `/fusion/status`. The Phase 1 instrument had no way
  to know, which is why it stayed quiet through four divergences.
- **LangGraph Studio brought up on the Pi 5** — see OPERATIONS.md §7, plus
  `~/ros2_ws/scripts/start_studio.sh` on the Pi. Studio now gets its own episodic store
  (`~/.langrobo/qdrant_studio`) instead of starting with none, and
  `STUDIO_MODEL` was corrected from `gpt-4o-mini` to `default` so traces stop
  claiming an OpenAI model when llama.cpp is serving Gemma.

## Open item 4 (ssh to the Pi 5 hangs) — a candidate cause, found by hitting it

Reproduced today: ping 0% loss, port 22 accepting TCP, teleop answering 200,
and eight consecutive `ssh` calls returning **exit 255 with no output at all**.

At least some of that is self-inflicted, and it is worth knowing before anyone
chases the network again. The command being run remotely was

```
pkill -f "langgraph dev"
```

`pkill -f` matches against the whole command line — **including the command line
of the shell sshd started to run it**, which contains the pattern text. So the
remote shell kills itself and its own session. Exit 255, nothing printed,
nothing actually killed, and it looks precisely like a flaky link. `pkill -f
"langgrap[h] dev"` fixes it: a regex that cannot match its own literal text.

This does **not** retire open item 4 — the 2026-09-06 report was during a fleet
check and nobody has confirmed a self-matching `pkill` was involved there. But
any future "ssh to the Pi 5 is hanging" should rule this out first, because the
signature is identical and the cause is local.

## Not verified this pass

No motion was commanded. A human watches every autonomous move and nobody was
standing over it, so "the stack is up and honest" is the claim; "it drove" is
not.
