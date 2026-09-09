# Power-on → working development environment

**The one document to follow after everything has been switched off.**

You powered down the D555, the Jetson, the Pi 5 and the ESP32. You have powered
them back on. This is what to run, in what order, what "good" looks like at each
step, and what to do physically when a step fails.

For *why* any of it is shaped this way see [ARCHITECTURE.md](ARCHITECTURE.md).
For the full runbook including calibration and troubleshooting see
[OPERATIONS.md](OPERATIONS.md).

---

## 0. The five boxes

| box | address | how you know it is alive |
|---|---|---|
| Jetson Orin Nano | 192.168.1.15 | you are on it; `docker ps` shows `rover` |
| D555 depth camera | 192.168.11.55 | **only** a live publisher on `infra1`. Ping proves nothing |
| ESP32 wheel link | via micro-ROS | `/wheel_state` at ~20 Hz |
| Pi 5 (teleop + brain) | 192.168.1.16 | `:8091/mode` answers, `agent_node` running |
| Mac mini (LLM/VLM) | 192.168.1.6 | `:8080/v1/models` answers |
| Laptop (RViz) | **DHCP — moves** | `./rover view` finds it |

Two traps worth knowing before you start:

- **The D555 answers ping while completely dead.** It is a PoE network device
  speaking DDS, not a USB camera. Never diagnose it with `lsusb` or `ping`.
- **`./rover status` on a cold container is slow, not broken.** It walks eleven
  topics at ~12 s each — over two minutes to print a table of dashes. Start a
  layer first; the layer gates are the fast answer.

---

## 1. Jetson — start the container

```bash
cd ~/rover
docker ps -a --format '{{.Names}}\t{{.Status}}'   # is `rover` there, and stopped?
docker start rover
```

After a power cycle the container is stopped, not gone. `docker start` is
enough; `./rover camera` will create one if it is genuinely missing.

---

## 2. The stack, one layer at a time

Run these **in order**. Each layer checks the one beneath it, so a failure names
its own layer instead of hiding in a wall of log. **Do not skip ahead when one
fails.**

```bash
./rover camera      # the D555 alone       — IR >=15 Hz, depth >=10 Hz
./rover pose        # + cuVSLAM and gyro   — VO >=10 Hz, gyro >=50 Hz
./rover fused       # + fusion -> /odom    — /odom >=15 Hz, AND the honesty check
./rover map         # + nvblox             — occupancy grid >=1 Hz
./rover nav         # + nav2               — both costmaps
./rover vlm         # + VLM pixel -> goal  — colour >=2 Hz, 1 listener
./rover view        # RViz on the laptop
```

**Do not stop at `nav`.** `vlm` is what lets the Pi 5 brain *see* — without it
`look()` and `approach_described_object()` are silently dead, the brain answers
questions but is blind, and `./rover status` shows the `vlm` row as `--`.

`vlm` costs you depth rate: ~24 Hz → ~14 Hz once `image_bridge` attaches, worst
case a 633 ms gap (measured 2026-09-09). Still over the 10 Hz gate. This is
consumer load, **not** `align_depth` — an earlier note blaming the driver's
reprojection was wrong.

**But the 10 Hz gate is not the constraint that matters here.** Later the same
day, under all six layers, the same stall was measured at **1.4–1.9 s** — past
cuVSLAM's `max_frame_delta_s` of 1.0 s, which resets tracking. `vo_node` logs
`frame gap` and the fusion node logs `vo DOWN`, both recovering in ~1 s, so no
rate gate ever sees it. See [TODO.md](TODO.md) §31.

### When a layer fails

| layer | what it means | what to do — physically |
|---|---|---|
| `camera` | the D555's on-camera DDS server is dead | **unplug its PoE cable ~5 s and replug.** No software restart recovers this |
| `pose` | camera is up but cuVSLAM is not tracking | needs texture. Do not start it facing a bare wall with the emitter off |
| `fused` | prints `✗ DIVERGED` | `./rover pose` **then** `./rover fused`. `fused` alone restarts fusion and NOT cuvslam, so the divergence survives |
| `map` | refuses to start | that is deliberate — the pose is dishonest. Fix the pose first, see above |
| `nav` | costmaps flat | the map has no obstacle data yet. Drive it a little |

---

## 3. The cuVSLAM honesty check — now automatic

cuVSLAM has diverged four times and **the fallback is silent**: the rover drops
to gyro-plus-wheel dead reckoning while `/odom` stays at 20 Hz, `ready: true`,
every rate gate green and nav2 planning and driving. One session ran 21 m that
way before anyone noticed.

**You no longer have to remember to check this.** As of 2026-09-09:

- `./rover fused` runs the check in its gate and prints a `cuvslam` line.
- `./rover map` **refuses** to build on a diverged pose — mapping while diverged
  corrupts the map permanently rather than degrading it.
- `./rover compare` shows a red banner live and in its verdict.

### It catches divergence. It does NOT catch a dead tracker.

The check reads `vo_z`, `vo_implausible`, `dr_metres` and `landmarks` — all
last-known values that **freeze** rather than disappear when `vo_node` stops
publishing. On 2026-09-09 it printed `ok — tracking, not dead reckoning` with
no cuVSLAM node running at all, and `./rover map` built on that pose without
refusing. `dr_metres` is the field that would have caught it, and it reads 0.00
until the rover actually drives — so a stationary rover with a dead tracker
looks exactly like a healthy one, which is the state every bring-up is in.

**So read the `/vo/odom` rate row next to the `cuvslam` line.** If it is `0.0`
and the `cuvslam` line still says `ok`, believe the rate. Fix is the same:
`./rover pose` **then** `./rover fused`. Full detail in [TODO.md](TODO.md) §30.

What you want to see:

```
cuvslam   vo_z +0.007 m   implausible 0   dead-reckoned 0.00 m   landmarks 132
          ok — tracking, not dead reckoning
```

Do **not** check this with `ros2 topic echo /fusion/status --once` — it truncates
the JSON mid-field, so anything grepping it silently reads nothing.

---

## 4. ESP32 — the wheel link

```bash
docker exec rover bash -lc 'unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE; \
  export ROS_DOMAIN_ID=0; source /opt/ros/jazzy/setup.bash; \
  ros2 topic hz /wheel_state; ros2 topic info /cmd_vel'
```

Want **~20 Hz** and `/cmd_vel` **Subscription count: 1**.

If it is 0 Hz or subs 0, the ESP32 did not reconnect its micro-ROS session —
**power-cycle the rover** (the board, not the Jetson). Old firmware does not
retry the agent connection. It usually comes back on its own after a power
cycle; it did on 2026-09-09 with no intervention.

---

## 5. Pi 5 — teleop and the brain

These start themselves at boot. **Check, do not start.**

```bash
ssh 192.168.1.16 'pgrep -af "teleop_we[b]|agent_[n]ode|brain_launc[h]"'
curl -s http://192.168.1.16:8091/mode          # want {"manual": false}
```

| check | want | if not |
|---|---|---|
| `teleop_web.py` running | 1 process | `ssh 192.168.1.16 'pkill -f teleop_web.py'` — `Restart=always` brings it back |
| `agent_node` running | 1–2 processes | it is launched by `brain_launch.py`; see OPERATIONS.md §7 |
| `:8091/mode` | `{"manual": false}` | **AUTO.** In MANUAL it streams zeros at 10 Hz that cancel nav2's commands |

The brackets in that `pgrep` are deliberate: without them the pattern matches
the command line of the shell running it, and every check reports one extra
"process" that is just itself. The same trap kills your ssh session outright
when the command is `pkill` — see §6.

**Teleop must be in AUTO before any autonomous motion.** In MANUAL the rover
crawls, stalls, and it looks exactly like a controller fault.

---

## 6. LangGraph Studio — the link you asked for

Not started at boot. `agent_node` runs the graph in-process and needs no server;
Studio is a **separate dev server** you start by hand.

The launcher lives at `~/ros2_ws/scripts/start_studio.sh` on the Pi 5, and a
version-controlled copy is in [`phase4/pi5/`](phase4/pi5/).

**The Pi 5 repo also has its own `./scripts/dev.sh`**, which predates this one
and does more: it starts a micro-ROS agent alongside Studio and uses the Fast
DDS Discovery Server on `127.0.0.1:11811` (see that repo's `NETWORKING.md`).
There is also `./scripts/dev_voice.sh`, which adds STT/TTS so you can talk to
the graph while stepping it.

Use `dev.sh` when the brain is **stopped** and Studio is your only client —
that is what it is written for, and its header warns against running it
alongside `langrobo-brain` because both drive `/cmd_vel` and both bind
micro-ROS UDP 8888. `start_studio.sh` is the variant for the situation we are
actually in: `agent_node` already running, so **no** micro-ROS agent, plain
SUBNET discovery to match the running brain, and a separate episodic store.
Read §6's warning about two `/cmd_vel` publishers before using either.

### Start it on the Pi 5

```bash
ssh 192.168.1.16 'setsid nohup ~/ros2_ws/scripts/start_studio.sh \
    > /tmp/langgraph_dev.log 2>&1 < /dev/null &'
```

It takes 30–60 s. Poll rather than assume:

```bash
ssh 192.168.1.16 'curl -s http://127.0.0.1:2024/ok'      # {"ok":true} when ready
```

### Open it — TWO steps, both required

**Step 1** — on the machine with the browser, leave this running:

```bash
ssh -N -L 2024:localhost:2024 rakhi24@192.168.1.16
```

**Step 2** — open:

```
https://smith.langchain.com/studio/?baseUrl=http://127.0.0.1:2024
```

**The link alone will not work.** Studio's page is served over HTTPS and a
browser blocks it from calling `http://192.168.1.16:2024` as mixed content, with
no visible error. `http://127.0.0.1` is the one address browsers exempt, and the
tunnel is what makes it `127.0.0.1` on your laptop.

Check the tunnel from your laptop: `curl localhost:2024/ok` → `{"ok":true}`.

### Stopping it

```bash
ssh 192.168.1.16 'pkill -f "langgrap[h] dev"'
```

**The brackets are not a typo.** `pkill -f "langgraph dev"` matches its own
command line over ssh and kills its own session: exit 255, no output, nothing
killed — indistinguishable from a flaky link.

### LangSmith is a different thing, and needs none of this

[smith.langchain.com](https://smith.langchain.com), project **`pi5`**. Traces of
real turns the rover has already taken through Telegram. No tunnel, no server,
nothing to start — tracing is on (`LANGCHAIN_TRACING_V2=true`).

For *reading* how agents hand off and which tools fire, LangSmith is the lower
effort path. Studio is for driving the graph yourself.

---

## 7. What "all good" looks like

`./rover status` — all eleven rows, plus an honest pose:

```
  camera  /camera/camera0/infra1/image_rect_raw   28.6   want 15   OK
  camera  /camera/camera0/depth/image_rect_raw    15.4   want 10   OK
  pose    /vo/odom                                21.5   want 10   OK
  pose    /gyro/base                             199.2   want 50   OK
  wheels  /wheel_state                            19.9   want 15   OK
  fused   /odom                                   20.0   want 15   OK
  map     /nvblox_node/static_occupancy_grid       4.8   want  1   OK
  nav     /global_costmap/costmap                  0.8   want0.5   OK
  nav     /local_costmap/costmap                   1.7   want  1   OK
  vlm     /camera/color/image_raw/compressed       4.5   want  2   OK
  view    /rover/model                             1.0   want0.5   OK

  cuvslam  vo_z +0.007 m  implausible 0  dead-reckoned 0.00 m  landmarks 132
           ok — tracking, not dead reckoning
```

Plus, off the Jetson:

- `/wheel_state` ~20 Hz, `/cmd_vel` subs 1 — ESP32 linked
- `:8091/mode` → `{"manual": false}` — teleop in AUTO
- `agent_node` running on the Pi 5 — brain up
- `:2024/ok` → `{"ok":true}` — Studio up, if you started it
- `ros2 node list` shows `/agent_node` and `/studio_bridge`

---

## 8. What this does NOT restore

Following this document gets you a **working rig**, not the *identical* state you
left. The difference is deliberate in some places and a known gap in others.

**Comes back exactly:**

- every layer and its rates, the honesty of the pose, the ESP32 link
- the Pi 5 brain and teleop — they start at boot
- Studio's own episodic store (`~/.langrobo/qdrant_studio` persists on disk)
- the Pi 5's `.env`, including `STUDIO_MODEL=default`

**Does NOT come back, by design:**

- **The map.** nvblox builds in the `odom` frame and is discarded on shutdown —
  phase 2c ("a map that survives a power cycle") is deferred by choice. You
  start with an empty grid and it grows as you drive.
- **The odom origin.** It resets to wherever the rover is when `./rover fused`
  starts. Saved locations (`kitchen`, `entrance`, …) are coordinates in that
  frame, so **they only mean anything relative to this session's start pose.**
  If you moved the rover while it was off, they point somewhere else.

**Backed up, but not in this repo:**

- **`~/ros2_ws` on the Pi 5 is its own git repo** —
  `github.com/Rakeshreddysr2401/pi5_ros2_ws`, branch `dev-1.2.8-refactor-test`.
  The whole `langrobo_core` / `langrobo_ros` brain tree is version-controlled
  there, so a reflash is a `git clone`, not a loss. (An earlier draft of this
  section claimed it was unbacked-up. It was wrong.)
- **`.env` is the exception, and correctly so** — it is gitignored on the Pi 5
  because it holds the LangSmith / Sarvam / Soniox / OpenAI keys. `example.env`
  is tracked as the template. **This is the one thing with no backup anywhere**:
  if that SD card dies you re-issue keys. Worth a copy somewhere safe.

**Varies between power cycles:**

- **Whether the ESP32 reconnects on its own.** It did on 2026-09-09 with no
  intervention; other sessions have needed a physical power-cycle. §4 tells you
  which you got.
- **The laptop's IP** — DHCP. `./rover view` finds it.
- **Studio is not started at boot** — §6 every time.

---

## 9. Before anything drives

- **A human watches, every time.** The rover is blind below 10 cm, above 24 cm,
  outside 87°, and **downward** — there is no drop-off detection at all.
- **Tapping MANUAL on the phone cancels the active nav2 goal.** That is the stop.
- **Studio can drive.** With ROS sourced, a `navigate` turn typed into the
  browser publishes real `/cmd_vel` — `/studio_bridge` sits alongside
  `/agent_node` and both can command the wheels. To explore the graph *without*
  that, start Studio in a shell where ROS is not sourced: it falls back to
  `StubBridge` and logs tool calls instead of publishing.
