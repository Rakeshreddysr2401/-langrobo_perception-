# Voice placement — why STT/TTS moved to the Pi5

Measured 2026-09-04 on this Jetson, branch `rover-v1.0.6`. Answers one
question: can voice (STT/TTS) run on this box while the autonomy stack
(cuVSLAM + fusion + nvblox + nav2) is driving. Short answer: no, and here's
the measurement, not a guess.

---

## The measurement

Brought up the full stack (`./rover camera` → `pose` → `fused` → `map` →
`nav`) and read the numbers live, camera and D555 powered, nothing simulated:

| | idle (IDE only) | **full autonomy stack** |
|---|---|---|
| RAM available | 3.5 GiB | **1.6 GiB** (147 MB truly free, swap touched) |
| GPU (GR3D) | 0% | **96–99%**, bursting |
| CPU, all 6 cores | 5–8% | **44–65% each**, clocked to 1497 MHz |
| VDD_IN | ~5 W | ~8.8 W (of the 15 W cap set — see JETSON_LOAD.md §1) |

This is the perception/nav loop alone — cuVSLAM, fusion, nvblox, nav2. No
voice model loaded. There is no spare GPU and barely any spare RAM for a
GPU-resident Whisper model on top of this.

This isn't a new finding — `~/robot/DEPTH_CAMERA.md` measured the same
conclusion on 2026-07-12 (voice stack alone: ~6.5GB, "cuVSLAM + nvblox will
not fit on top") and it's wired into `~/robot/scripts/fleet_role.sh`
("perception has priority on the 8GB Orin, voice OFF while perception
runs"). What changed: cuVSLAM was parked back then (Thor/Orin binary
mismatch) and is now live and running full-tilt — today's Jetson has *less*
headroom than July's, not more. The old conclusion holds, harder than before.

---

## The decision

Perception stays on the Jetson — not a choice, cuVSLAM/nvblox are CUDA
kernels, this is the only GPU on the vehicle. Voice does not share this box
while driving. Two channels now exist for talking to the robot while it's
in motion:

1. **Telegram** — already built, already working, full agent access, no new
   engineering (`~/robot`'s Pi5 counterpart documents this: `TELEGRAM.md`).
2. **A second, independent STT/TTS pair on the Pi5's own CPU** — built
   2026-09-04, `pi5_voice_pkg` in the Pi5 repo (`~/ros2_ws` there). Not a
   port of the Jetson's `ai_stack` — a smaller, CPU-only build:
   `faster-whisper` base/int8 (measured RTF ~0.75, faster than real time on
   the Pi5's Cortex-A76) + `kokoro-onnx` fp32 (measured RTF ~1.8 — slower
   than real time, but the wire protocol already sends one sentence per
   message, so the delay doesn't compound). Full writeup, measured numbers,
   and verified-vs-pending status: `PI5_VOICE.md` in that repo.

The Jetson's own voice stack (`ai_stack`, CUDA Whisper + Kokoro) is not
retired — it's still the better choice when the robot is parked, and stays
the documented Telegram-off fallback (`fleet_role.sh voice start`) for that
case.

---

## Reproduce

```bash
cd ~/rover
free -h && ./rover camera && ./rover pose && ./rover fused && ./rover map && ./rover nav
free -h; tegrastats --interval 1000    # kill after a few lines
./rover stop                            # tear down — don't leave nav2 live unattended
```
