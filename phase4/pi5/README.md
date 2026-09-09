# Pi 5 — LangGraph Studio launcher

**This runs on the Pi 5, not the Jetson.** It is copied here so it is
version-controlled; the live copy is `~/ros2_ws/start_studio.sh` on
`192.168.1.16`.

`STARTUP.md` §6 and `OPERATIONS.md` §7 both tell you to run it, so if the Pi 5
is ever reflashed this file is what puts it back:

```bash
scp phase4/pi5/start_studio.sh 192.168.1.16:~/ros2_ws/start_studio.sh
ssh 192.168.1.16 'chmod +x ~/ros2_ws/start_studio.sh'
```

## It is not the only launcher — read this first

The Pi 5 repo (`github.com/Rakeshreddysr2401/pi5_ros2_ws`) already ships
`./scripts/dev.sh` and `./scripts/dev_voice.sh`, and they predate this script.
`dev.sh` starts a micro-ROS agent alongside Studio and uses the Fast DDS
Discovery Server at `127.0.0.1:11811`; `dev_voice.sh` adds STT/TTS so you can
speak to the graph while stepping it.

| | `scripts/dev.sh` | `start_studio.sh` |
|---|---|---|
| micro-ROS agent | starts one (UDP 8888) | none — assumes one is already up |
| discovery | `ROS_DISCOVERY_SERVER=127.0.0.1:11811` | plain SUBNET, matching the running `agent_node` |
| episodic memory | default store — **empty if the brain holds it** | its own `~/.langrobo/qdrant_studio` |
| written for | brain **stopped**, Studio the only client | brain **already running** |

`dev.sh`'s own header warns against running it while `langrobo-brain` is up —
both drive `/cmd_vel` and both bind micro-ROS UDP 8888. That warning applies to
this script too for `/cmd_vel`: with `agent_node` running you get two publishers.
Deliberate here, but know it.

**If you are stopping the brain to work in Studio, use `dev.sh`** — it is the
richer, older, better-integrated path. This script exists for observing the live
robot without disturbing it.

## What it starts

`langgraph dev`, serving the agent graph on `127.0.0.1:2024` — the same graph
`agent_node` runs in-process, but over HTTP so LangGraph Studio can drive it.
Nothing starts it at boot; the robot does not need it.

## Every line in it is load-bearing

Each of these fails **quietly** when wrong, which is why they are in a script
rather than in someone's shell history:

- **`cd ~/ros2_ws`** — the CLI reads `langgraph.json` and `.env` from the working
  directory, not from its own location.
- **Full path to `~/.local/bin/langgraph`** — it is not on a non-interactive ssh
  PATH, so `ssh pi5 'langgraph dev'` is `command not found`.
- **ROS vars matched to `agent_node`'s** (domain 0, SUBNET discovery). A stale
  `ROS_DISCOVERY_SERVER` gives a bridge that starts perfectly cleanly and
  silently sees no robot at all.
- **`LANGROBO_MEMORY_PATH` → `~/.langrobo/qdrant_studio`**, and deliberately
  **not** in `.env`: `agent_node` reads that same file, so setting it there
  would repoint the *robot's* memory. Embedded Qdrant is single-process, so
  without a separate path Studio starts with no episodic memory at all.
- **No `set -u`** — ROS's `setup.bash` references unset variables and aborts the
  script the instant it is sourced, silently if its stderr is going to
  `/dev/null`.
- **`--allow-blocking`** — the graph does synchronous ROS and HTTP work and the
  server otherwise raises on the first blocking call.

## It can drive the rover

`graph_studio.py` attaches a real `ROS2Bridge` whenever ROS is sourced, so
`/studio_bridge` appears next to `/agent_node` and a `navigate` turn typed into
the browser publishes real `/cmd_vel`. To explore the graph without that, run
`langgraph dev` from a shell where ROS is **not** sourced — it falls back to
`StubBridge` and logs tool calls instead of publishing.

## Stopping it

```bash
ssh 192.168.1.16 'pkill -f "langgrap[h] dev"'
```

The brackets are not a typo. `pkill -f "langgraph dev"` matches its own command
line over ssh and kills its own session: exit 255, no output, nothing killed.
