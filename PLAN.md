Goal: To Make My Rover To Acheive Its Goal In Know and Unknown Environment


Phase1: Use what ever sensor data you like visual odometry(cuVslam,nvblox), acce/gyro(from depth sensors), encoders data(from all 4 motors)
        I am not sure from which we collecting what and whats the final output , but according to my knowledge will get some x(cm),y(cm),teta(degree) changes
        phase1 goal: Integerate what ever required , i will move my robot using manual control/by hand i need to see those values changing you can log
        phase1 human help: I will help you in adjusting your values like you ask me how much it moved or ask me to any physical check

By Phase1 one You Have Complete movement control perception in sync

Phase2:Map or geting aware of environment creating walls and maps

Phase3:Path Planing 

Phase4:Motion Planing

Here in Path 2,3,4 will also need to work on unknow new places will see them later

Hardware: we have: Depth Camera,Pi5 , Laptop(Rviz) and jetson orin nano and mac mini(vlm currently not active) and rover with 4 encoded motors

Important: have a clean and modular code easy to extend and integrate and understandable

---

# Settled — 2026-08-15

Decisions taken in the design interview. Everything above is the goal in my own
words; this is what it turned into.

## Ground rules

- **This file is the plan of record.** The previous `PRD.md`, `tasks/`,
  `FACTS.md` and `src/` were deleted; nothing is inherited from them. They are
  recoverable from commit `dc05aa0` if ever wanted.
- Code runs inside `orin-nav:1.1`. That image has **no build recipe and cannot be
  reproduced** — it was made by `docker commit`, not from a Dockerfile. Never
  install into it; never delete it.
- Python, ROS 2 Jazzy. **SI internally** (metres, radians, so nvblox and nav2 can
  consume it later); centimetres and degrees on screen, per Phase 1's wording.
- Bring-up is **layered** — one command per layer, each printing its own
  PASS/FAIL — so a failure names its own layer instead of hiding in a wall of log.

## Phase 1 — what it actually is

**Side-by-side, not fused.** Each sensor reports separately and you referee with
a tape measure. A single fused pose cannot tell you which input is lying: when
cuVSLAM under-read a 100 cm push as 26 cm, a fused number would have looked
entirely plausible. Fusion is Phase 1b, once we know which sources have earned
weight.

Sources, and what each can honestly contribute:

| source | gives | state |
|---|---|---|
| stereo IR → cuVSLAM | x, y, θ | **working**, 27–29 Hz |
| gyro (inside the D555) | θ̇ only | **working**, 200 Hz |
| wheel encoders | vx, and θ̇ from velL−velR | **blocked** — needs a reflash |
| accelerometer | nothing usable | diverges by hundreds of metres if integrated |

Two corrections to the phase as first written:

- **`/odom` is not a separate sensor.** It is cuVSLAM's output, and cuVSLAM's
  input is the D555's stereo IR. "D555 + odom" is one source, not two.
- **nvblox is not odometry.** It *consumes* pose to build a map. It belongs to
  Phase 2.
- **"encoders from all 4 motors" does not reach ROS.** The firmware reads all
  four (`encLF/LR/RF/RR`) but averages them per side and publishes
  `Vector3(velL, velR, cmd_vx)`; full odometry "exceeds the micro-ROS WiFi
  message size". Per-wheel data would need a firmware change.

## Phase 1 gate

Hand-pushed, all three:

1. **scale** — push 2.00 m straight → reads 1.90–2.10 m
2. **drift** — 2.00 m out and back to the mark → ends ≤ 0.10 m from start
3. **heading** — rotate 360° by hand → heading error ≤ 10°

Separately, **prove teleop**: `/cmd_vel` moves the wheels, and a stop command
stops them. Gate measurements stay hand-pushed because motor-driven wheels slip
and cannot hold a tape-exact 2.00 m under PID.

Why three and not one: scale, drift and heading fail differently. A single
straight push only tests scale, and a perfect scale factor can still smear a map
if error accumulates as you move.

## Open, not yet decided

- **Phase 2 persistence** — must a map survive being switched off? It is a frame
  decision (`map` vs `odom`) and expensive to retrofit, so it needs settling
  before Phase 2 code exists. Both packages support it: cuVSLAM has
  `LocalizeInMap`/`FilePath` services, nvblox has `FilePath`.
- **Phase 3 vs Phase 4** — "path planning" and "motion planning" are listed as
  separate phases, but in nav2 they are the planner and the controller and they
  arrive together. Worth checking whether that seam is real.
- The mac mini VLM, and what "intelligently" means beyond autonomy.

