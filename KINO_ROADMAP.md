# Kino: what exists, and what to improve (2026-10-16)

Reading of "a real personal/domestic robot": it hears you, answers fast, knows
where it is and where things are, moves safely among people and furniture, runs
all day without babysitting, and has some character. Sources: SYSTEM_OVERVIEW.md,
OPEN_ISSUES.md, LOCALIZATION_GAPS.md, NAV_GUIDE.md, ROVER_BUILD_PLAN.md.

## What exists

| layer | machine | state |
|---|---|---|
| Body: 4-wheel ESP32 base, RPLidar C1, D555 (depth + gyro) | Jetson / ESP32 | built, up 2026-10-11 |
| Pose: cuVSLAM + gyro + wheels fused (`fusion2`), slam_toolbox, nav2, nvblox | Jetson | works; ~1 cm goal; tight gaps down to ~50 cm |
| Eyes: YOLO people, VLM grounding (pixel to goal), photo log, follow-me | Jetson + Mac mini | working, floor-tested |
| Brain: LangGraph (chat / local_agent / navigate), memory, Telegram | Pi 5 | working |
| Voice: STT (Whisper/Parakeet/Apple), TTS, wake aliases | Pi 5 + Mac mini | working, no acoustic wake word yet |
| Twin: Gazebo copy of the robot on the laptop | laptop | parked |

## What stops it feeling like a real robot (most important first)

1. **Reliability.** Ordinary use still needs a human: WiFi on the Jetson dropped
   to -72 dBm, depth fell to 4.9 Hz, ssh hung; the Mac mini moves IP on DHCP; a
   clicked goal took 551 s and failed (OPEN_ISSUES 2026-10-11). Fix: wired
   Ethernet or a good AP, fixed IPs / `.local` names, a watchdog that restarts
   dead layers, one health page.
2. **Power.** Battery sag changes turn drift 6-33 cm per 90 degrees. Redesign
   batteries and connections (ROVER_BUILD_PLAN.md 4.5) before more motion work.
3. **Memory of the home.** The map must survive a power cycle (owner wants it
   saved). Then named rooms and places, "go to the kitchen" from a cold start,
   relocalise on boot.
4. **Navigation you can trust.** Camera-only phantom obstacles, 44 cm gaps,
   slow recovery. Finish NAV_GUIDE F1-F5; LiDAR first (sensor strategy). Add
   cliff / bump sensing and a hard stop that does not depend on the network.
5. **Voice that feels instant.** Train the Kino acoustic wake word (needs the
   model, then `wake_switch.py`), stream TTS, answer within about a second for
   simple things, barge-in (talk over it to stop it), speak-first before slow
   tools. Today a 12B model on a Mac mini is the latency floor.
6. **Awareness of people.** Follow-me exists. Add recognising household members
   (face or voice, with consent), turning to face the speaker, not driving into
   people, a "who is home" memory. A track is not identity today.
7. **Proactivity.** Reminders, timers and Telegram exist. Missing: patrol,
   "check if the door is closed", "where did I leave my keys" from the photo
   log, low-battery self-docking.
8. **Character.** A face or LED ring and sounds (heard, thinking, error), idle
   behaviour, a consistent voice and short speech. Kino has a name and a prompt;
   it has no body language yet.
9. **Self-docking and charging.** The single biggest gap between a project and a
   product: it cannot recharge itself.
10. **Safety and privacy.** Camera and mic indicator, a physical mute, local-only
    processing by default, a clear e-stop, speed limits near people.
11. **Later:** parking assistant, an arm (README order), omni wheels.

## Suggested order

1. Network + power + watchdog (reliability).
2. Persistent map and relocalise on boot.
3. Nav fixes F1-F5, safety stop.
4. Kino wake word + faster voice.
5. Docking.
6. People identity, then proactive tasks, then character.

## Technical upgrades, like YOLO was (2026-10-16)

Candidates, ordered by payoff per effort. Verify versions and Orin Nano
(8 GB, shared with nvblox + cuVSLAM) fit before adopting any.

| area | add | replaces / fixes |
|---|---|---|
| Wake word | openWakeWord "Kino" model (or Porcupine) | name-in-text gate; always-on STT |
| Speech in | Silero VAD + streaming ASR (sherpa-onnx Zipformer), echo cancel (WebRTC AEC / speexdsp) | batch STT; robot hearing itself |
| Speech out | Piper/Kokoro streaming TTS, start on first sentence | wait-for-full-reply |
| LLM | speculative decoding / smaller router model (1-4B) for routing + chat, big model only for vision/planning; prompt-cache pinning (already) | 12B for every turn |
| Intent | embedding router (exists) + grammar/JSON-constrained tool calls | invalid tool calls |
| Detection | YOLO tracking (ByteTrack) + TensorRT FP16/INT8; open-vocab (YOLO-World / OWL-ViT / Grounding DINO) for "the red bottle" | VLM box on every grounding call |
| People | face / voice embeddings (InsightFace, ECAPA) stored with consent | track != identity |
| Mapping | save/load slam_toolbox posegraph; AMCL or slam localization mode on boot; semantic map (object -> pose) in SQLite | map lost on power cycle |
| Nav | LiDAR-only costmap layer first; MPPI or Smac + RPP tuning; nav2 behavior tree w/ short recovery; cliff/bump sensors; ESP32 hard watchdog (exists) | phantom camera obstacles, 1-min recoveries |
| Depth | depth-confidence filtering, temporal filter; DA-V2 mono depth as a fallback only | blur/noise |
| Memory | vector store (sqlite-vec / Chroma) of photo captions + CLIP embeddings; retrieve, not dump | linear photo log |
| Network | Ethernet backbone, CycloneDDS or Zenoh with unicast peers, compress images (JPEG/zstd), QoS tuning | multicast over weak WiFi |
| Ops | systemd units + watchdog, health endpoint, Prometheus/Grafana or simple status page, structured JSON logs, rosbag ring buffer for failures | manual `rover up` |
| Testing | rosbag replay CI, pytest + the Gazebo twin as regression, golden transcripts for the prompts (eval set of 50 utterances) | "it worked yesterday" |
| Dev | pre-commit (ruff), type checks, one `docker compose`, config in one place, Studio traces (LangSmith/Langfuse) | scattered config |
| Power | INA219/INA226 current+voltage sensing on the ESP32, brownout warnings, battery % into the prompt | battery sag surprises |
