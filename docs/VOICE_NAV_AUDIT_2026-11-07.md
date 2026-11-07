# Voice and navigation audit — 2026-11-07

Branch: `fix/voice-telegram-navigation-2026-11-07` in this repo and the Pi5 `~/ros2_ws`.

## What the live logs establish

- The early Pi5 forward request used timed driving while goal_exec was still starting. The Pi5 bridge now returns `not_ready` rather than enabling its offline timed fallback. Stop is preserved. No Jetson motion protocol changed.
- The reverse request at goal_exec timestamp 1794065773 was refused: an obstacle was reported 0.63 m into a 0.85 m leg. This does not establish a false obstacle. The Pi5 model had requested 50 cm for "come little back"; small-move and clarification behavior is being checked with the real model on the Pi5.
- Following at 1794066249-1794066253 found nobody, then reported no short escape move that opened space (10.2 cm free). No obstacle margin or escape limit was relaxed.
- RL is BUILT and ENABLED, not merely planned: `logs/rl_mode` is 1; the running reach command has `REACH_RL=1`, policy `/models/rl/current/policy.npz` (v002).
- The recovery for goal stamp `1794067053.372403849` entered RL at 1794067073.4. Its intermediate target was reached in 8.3 s / 70 steps; the shield changed 10 commands; logged minimum clearance was 3.8 cm. Reach then finished with normal navigation: final reported error 7.3 cm in 42 s total. These are estimates/logged results, not a tape-measure or collision-ground-truth grade.
- The policy metadata `succ: 0.8470125882889668` is NOT a measured real-floor success rate.

## Checks run without commanding the real wheels

- `python3 phase3/tools/test_follow.py`: all 24 offline follow cases PASS, including wall-corner escape, boxed-in refusal, loss/reacquisition, and stopping as clearance closes.
- Pure `rl_drive` smoke check INSIDE the current rover container: deployed policy loads, observation/action dimensions match, inference is finite, a free command passes the shield, and an occupied outline produces stop. No ROS node or publisher was constructed.
- Pi5 `./scripts/fleet.sh check`: ALL OK (pose ~20 Hz, LiDAR ~10 Hz, wheels ~20 Hz, camera >4 Hz, grounding, exact moves, reach/Nav2, follow, slam, five Mac slots, Studio).
- The fleet process check now recognizes the Isaac `vslam_container`; the old `vo_node`-only name falsely reported it down.

## Remaining work

- Sarvam HTTP 402 / no credits: Piper fallback is working; topping up is external to code. Pi5 `/status` now exposes the observed voice provider and fallback reason.
- Original microphone recordings are needed to judge whether "call the door", "Me", and "Started" were transcription errors. STT/audio processing was not changed.
- Re-grade reverse clearances and follow behavior against a measured real scene before changing map thresholds or training/replacing the policy. No new floor drive was issued by this audit.
- Pi5 details and final validation: `~/ros2_ws/docs/voice/VOICE_NAV_FIXES_2026-11-07.md`.
