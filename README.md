# LangRobo

An indoor rover that maps a room and navigates it on its own: a Jetson Orin Nano
running cuVSLAM + nvblox + nav2, a Pi 5 brain, and an ESP32 driving the wheels.

## 👉 Start here: [`learn/`](learn/)

`learn/` is the working plan and the documentation. It takes you from "can I see
anything?" to "click a goal in RViz and the rover drives there", in nine steps
that each end in something you watched work.

| | |
|---|---|
| [`learn/README.md`](learn/README.md) | The nine issues, in order. **Start here.** |
| [`learn/PRD.md`](learn/PRD.md) | What we're building and what counts as done |
| [`learn/ARCHITECTURE.md`](learn/ARCHITECTURE.md) | How the pieces connect, and every measurement we have |
| [`learn/PROGRESS.md`](learn/PROGRESS.md) | Where the work is right now |

## The machines

```
D555 camera ──eth──► Jetson Orin Nano ◄──── WiFi (192.168.1.x) ────► Pi 5 ◄────► Mac mini
(192.168.11.55)      cuVSLAM · nvblox                              LangGraph    llama.cpp
                     nav2 · YOLO                                   Telegram     Gemma VLM
                           │                                       micro-ROS
                           │  planner → MPPI → smoother →              │
                           │  collision_monitor → safety_guard         │
                           └──────────► /cmd_vel ──────────► ESP32 (rover-esp32.local)
                                                              BTS7960 + encoder PID

     Your laptop (192.168.1.10) runs RViz — the window into all of it.
```

> ⚠️ The laptop is **`.10`**. DHCP gave `.12` to the ESP32, so ssh to `.12` reaches
> a microcontroller and looks exactly like a laptop that is switched off.

## Repo map

| Path | What |
|---|---|
| [`learn/`](learn/) | **The plan and the docs. Read this.** |
| [`orin-nav-stack/`](orin-nav-stack/) | The Jetson stack — `run_stack.sh`, nodes, configs, ESP32 firmware |
| [`orin-nav-stack/SYSTEM_INTEGRATION.md`](orin-nav-stack/SYSTEM_INTEGRATION.md) | Cross-machine ROS 2 contract + long-term gap roadmap |
| [`pi5/`](pi5/) | Pi 5 side: phone teleop web page, client CLI |
| [`archive/`](archive/) | The old RTAB-Map-era stack and superseded docs. History only — nothing in there runs. |

## Daily operation

Everything runs through one script on the Jetson:

```bash
cd ~/langrobo_perception/orin-nav-stack
./run_stack.sh up        # camera + TF + cuVSLAM (full SLAM) + nvblox
./run_stack.sh status    # health: rates, TF freshness, pose trust, Orin load
./run_stack.sh view      # check the laptop is ready for RViz, and say why if not
./run_stack.sh vision    # YOLO + goal grounding + safety guard
./run_stack.sh nav2      # autonomous navigation
./run_stack.sh stop      # E-STOP: kill motion nodes and zero /cmd_vel
```

Drive by hand from a phone on the same wifi: **`http://192.168.1.16:8091`**
(hold-to-move). See [`pi5/teleop/`](pi5/teleop/README.md).

> ⚠️ Phone teleop publishes straight to `/cmd_vel`, **bypassing `safety_guard`**.
> When you drive by phone, you are the safety system.

## Three things that will save you a session

1. **The D555 answers ping even when it is dead.** If the log says "No RealSense
   devices were found", only a **physical PoE power-cycle** recovers it.
2. **CPU is a safety property.** Above Orin load ~8 the camera starves, and
   cuVSLAM freezes **silently and permanently** while the stack keeps navigating
   on a frozen pose. Check `./run_stack.sh status`.
3. **This system fails silently.** Rates collapse, numbers come out confidently
   wrong, and nothing logs an error. That is why every step in `learn/` ends in a
   measured gate.
