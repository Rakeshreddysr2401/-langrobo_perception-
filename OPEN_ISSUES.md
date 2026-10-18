# Open issues — where things stand, 2026-10-09

How the rover plans and moves, why it is limited, and the fix order F1-F5: [NAV_GUIDE.md](NAV_GUIDE.md) (2026-10-11).

## 2026-10-18 — the twin moves to Isaac Sim (new laptop)

The twin runs THIS code on simulated hardware again (mitra_sim, Isaac Sim 5.1;
SYSTEM_OVERVIEW.md §3). Found on it, for the real robot:

| item | finding | status |
|---|---|---|
| door clearance | home_real goal 2 passed the 80 cm door frame at **0.7 cm** (2.8 cm in the morning run, 3.5 cm on the 10-11 twin): no touch, but no margin | open: look at nav2's inflation / the 1 cm hard margin in doors |
| the 1 cm tablet | driven over on goals 1 and 4, as on the 10-11 twin (below the lidar plane and nvblox's slice) | known (#3) |
| leaving a corner | home_real, from (0.3, -0.45, 180 deg) by the shoe rack / fridge (where the tour ends) to (3.05, 0.1): nav2 "no path: planner found none", `tight` "no fit found in the time" (3 s, 2837 expanded), `way_back` timed out twice ("stuck: the controller made no progress"), then `pass_far` got out after ~250 s; attempt 2 was on its way when cancelled at 298 s. | open: why no plan from there (the corner's costmap, the 3 s tight budget); the twin reproduces it: `./mitra scenario home_real`, then `TWIN=1 ./rover reach 3.05 0.1 0` |
| twin vs slam resume | `./rover stop` saves the room and `./rover slam` resumed it, but the twin always resets the rover to its world's start | fixed: `SLAM_FRESH` defaults to 1 under `TWIN=1` |
| twin vs laptop ssh | camera/lidar/reset ssh-ed into the laptop (`rakhi24`, `/workspace`): a new laptop broke it | fixed: the twin announces itself on `/sim/hardware`; `/sim/reset` |

## 2026-10-11 — a clicked goal with room to spare took 551 s and failed

Goal 2.47 m away through a ~60-70 cm corridor (the body needs ~44 cm).
`reach`: nav2 "no valid path" x4 with back-up + wait (~60 s), tight route (one
turn timed out at 30 s), then gap pass: **2 x 240 s in a turn**, 47 deg off.

| item | finding | status |
|---|---|---|
| **pass turn deadlock** (goal_exec_node) | in pass mode it stopped whenever the depth view was > 1.0 s old; depth_gate passes nothing while the rover rotates (blur), so every turn went stop / refresh / creep / stop. Each wait pushed the stall clock on, so it never fired; only gap_pass's 240 s cap ended it | **fixed:** turns no longer wait for a fresh view (LiDAR + nvblox memory; cancel only if > 15 s old); all depth waits of a goal capped at 20 s. goal_exec restarted 04:26 UTC. **Re-click the same goal to verify** |
| **camera-only lethal cells** | global costmap: 8206 lethal cells, **7695 nvblox-only**, 511 shared with the LiDAR; diagonal streaks and marks behind LiDAR walls; 14 % of cells free, 40 % inscribed. Same start poses plan fine minutes later (0.1-1.9 s), so the corridor was blocked when clicked | open -- tape tests with `phase3/tools/measure.py`, then depth filtering / camera-mark clearing |
| costmap publish rates | global 0.73 Hz (set 1.0), local 1.67 Hz (set 2.0), Jetson load 13-17 | open |
| nav2 recovery | 4 x (back-up + 5 s wait) before reach's own fallbacks: ~1 min per attempt | open, optional |

## 2026-10-09 — bring-up after the rebuild (ESP32 on USB), and a weak-WiFi evening

`./rover up` from cold: every layer gate passed (camera -> vlm), wheels 20 Hz over the
ESP32's new USB cable to the Pi 5. Then the Jetson's WiFi decided the evening.

| item | finding | status |
|---|---|---|
| **Jetson WiFi** | `Airtel_Singireddy's` 5 GHz, **-72 dBm (47 %), tx 6 Mbit/s**, sending ~2.3 Mbit/s of ROS data. ssh logins 15-30 s, ping up to 1.3 s, then ~1 min with no route at all. From the Pi 5 `/scan`, `/odom`, colour all 0 Hz while the Jetson's own rates were fine. Recovered by itself (~40 ms ping): every `fleet.sh check` line OK | the cure is signal (robot / AP placement) or the Pi 5 <-> Jetson cable (2026-10-07: jumbo-clean) |
| depth while the link was bad | **4.9 Hz** at the Jetson (IR 21.8, load 20); **18 Hz** once the WiFi recovered (load 16). Writers blocked on an unreachable reader is the likely cost -- not proven | re-measure on a healthy link; issue 1 stands |
| `rakhi-jetson.local` lost after the power cycle | avahi; `sudo systemctl restart avahi-daemon` on the Jetson (passwordless) fixed it; one later miss from the Pi 5 was the dead link | known (Pi 5 robot-start skill) |
| `./rover view`: laptop at a new address (.5) never found | host key checked per IP -> unknown host -> BatchMode refused. **Fixed:** keyed by the alias `rover-laptop` (OPERATIONS.md §5 "The laptop key") | fixed, tested from scratch |
| `./rover view`: exits silently after "config synced" | two `$(ssh ...)` under `set -eo pipefail` with a failing last command (`pgrep` / `ls` finding nothing) -- the on-demand-Xwayland path (TODO 35) could never run. Same trap as #9 | fixed (`|| true`) |
| `./rover view`: "nobody is logged in" on a timed-out ssh; `[pi5]`: "agent_node not found" on a timed-out ssh | an empty answer read as a fact. Both now say SSH_OK first, so a timeout says "did not answer", and timeouts are 15 s | fixed |
| `./rover view --restart` restarted nothing | `pkill` + `sleep 2`; rviz2 takes longer to exit, so "already running". Now waits until gone (SIGKILL at 10 s) | fixed, tested |
| wheel encoders, parked | `/wheel_ticks` + `/wheel_state` 20 Hz at the Jetson, four steady counts, fusion2 silent. Directions only show on a nudge | owner: `./rover wheels --nudge` with the rover watched |

## 2026-10-07 — bench mode: rover taken apart for cleaning (same build returns)

| item | finding | status |
|---|---|---|
| D555, direct cable to the Jetson | no switch: `enP8p1s0` MTU 9000, IR 29.9 Hz, depth 29.9 Hz, emitter off. Confirms the Waveshare switch was the jumbo-frame fault | works |
| LiDAR, ESP32, BT speaker | not connected (taken apart) | `./rover up` stops at `lidar` by design until the rebuild |
| camera to brain without a pose | the `vlm` layer refuses without `/odom`, so `image_bridge.py` was started by hand: colour 4.6 Hz received on the Pi 5. `look()` works; nothing can drive | bench only |
| PyCharm Gateway, MacBook (.6) to Pi 5 | Pi side is fine: the 2026-10-06 21:01 session connected and opened `~/ros2_ws`; it ended at the power cut (log ends in NULs). `.idea` files checked, not corrupted. The Pi announces itself as `rakhi24-desktop` (resolves only as `.local`), and an old Gateway entry pointed at the dead .22 | **owner, on the Mac:** key login + `Host pi5` (HostName 192.168.1.16) in `~/.ssh/config`, delete old Gateway entries, connect with key pair to 192.168.1.16, PyCharm 252.26830.27, `/home/rakhi24/ros2_ws` |

## TRIAL in place: LiDAR grounding for doors / walls (G1) -- verify or remove

**What it does:** when the brain asks to approach a door, wall, cupboard, wardrobe,
fridge or sofa, `pixel_to_goal` measures it with the LiDAR (the nearest continuous
surface across the middle of the VLM box's bearing) instead of the camera.
Everything else is unchanged.

| proven | not proven yet |
|---|---|
| 16 offline tests; live "the wall" under the desk: 1.24 m, past the legs and the bin (camera said 1.08) | **the real door** (closed, open, at an angle); a full "go near the door" drive through it |

Known weak spots: an **open** door (the LiDAR may measure the next room); a wall
with furniture in front (takes the furniture); only `approach` sends `what=`
(`locate.py`, `photo_recall.py` do not).

**Test:** rover ~2 m facing the closed door, then
`docker exec rover bash -lc 'source /opt/ros/jazzy/setup.bash; python3 /opt/rover4/tools/ground_check.py "the door"'`
(never drives) and compare with the LiDAR; again at an angle and with the door open.

**If it does not work:**
- *Turn it off:* `LIDAR_GROUNDING = False` in `phase4/nodes/pixel_to_goal.py`, then `./rover vlm`.
- *Remove it:* `git revert ad1319f 9fda655` (Jetson) and, on the Pi 5,
  `git revert 72e0bae` + restart the brain (the Pi side alone is harmless: the
  Jetson ignores `what=` when the trial is off or reverted).

**Keep regardless** (proven tonight): the camera re-calibration (add9474) and
D1 -- ignore specks, prefer the middle of the box (866a97d, 1dc6d96).

## 2026-10-04 (evening) — still open after the day's fixes

The day fixed S1-S5, S7, S10, the ghost marks, the RViz colours and the lost
CLI goals (sections below). **Tested live through Studio's API** (Pi 5
`/tmp/studio_test.py`, the same thread calls as the Studio page): "where are
you?" answered from the robot (B, after a wording fix, Pi 5 41dd3e4); "go near
my dad" asked what he looks like and did not move (E); "send me a picture"
went to the owner (C). The ghost rule and the blocker view were floor-tested
with the owner. Unit tests: Pi 5 492, Jetson trail 7 + ghost 8 + blocker 4 +
obstacle suite. What is left, most useful first:

| # | what | evidence | next |
|---|---|---|---|
| E1 | ~~Studio arrival report not yet seen live~~ **passed 2026-10-04** | Studio "go near the box": drove (1.1 m), and 22 s later "[SYSTEM] Navigation succeeded ... I can see the box ... 0.5 m away" arrived in the Studio thread by itself; "where are you?" then said "arrived 37 s ago". Cosmetic: the arrival check reads "the box in front of me in front of me" (the description is quoted inside a sentence that says it again) | trim "in front of me"-style phrases in `approach.arrival_check` |
| E2 | **The way back, after its fixes, not re-run** | live 2026-10-04: started as designed (2 hops, 1.41 m nearer), then a real tight spot; the 45 s hop limit and the honest note came after | a blocked-doorway drive with the owner; expect a goal over in < MAX_S and a note that matches the failure |
| E3 | **S6: a small turn stalls beside furniture** | a -27 deg turn beside the cot moved 1.5 deg, twice (goal_exec "turn stuck") | floor test with the rover beside the cot; swing check vs deadband |
| E4 | **The Jetson is overloaded** (= issue 1) | load 12-14 on 6 cores today; nav2 needed two bringups; depth stopped 7.8 s mid-approach; ghost_clear adds ~15% of a core | the heavy Python nodes first (fusion2, lidar_odom); nvpmodel is the owner's call (power supply) |
| E5 | **The vision model is slow** (= issue 3) | camera turns took 83-286 s in the 2026-10-04 conversation | Mac side: model / server flags (`--swa-full`, speculative decoding) |
| E6 | **People cannot be recognised** | "go to my dad" now asks what he looks like (S5) -- safe, not smart | a face/person-ID step, owner's choice whether to build it |
| E7 | **S8: corrections are not remembered** | "that screen is a cot" was only acknowledged | label a photo-log entry from a correction |
| E8 | **ghost_clear at walls** | a few marks right AT walls 2+ m away were erased (a slanting wall's return from beyond them) | if the rover ever touches a low thing against a far wall: `ROVER_GHOST_CLEAR=off` and report |
| E9 | **A LiDAR-layer mark in a shadow stays** | 1.4 m behind the rover, with something at 1.15 m in front of it: no beam reaches it | correct (nothing can say it is gone); clears when the blocker moves or the rover turns |
| E10 | **The Bluetooth speaker** | `br-connection-page-timeout` after the power cut: off or out of range | switch it on; the voice service picks it up |
| E11 | **Not pushed** | Jetson `rover-v1.1.7` 11 ahead, Pi 5 `dev-1.5.0-voice` 3 ahead of origin | push when the owner says |
| E13 | **ssh to the Pi 5 stalls at login** | three times on 2026-10-04: the Pi answers ping, teleop and the ssh banner, but key authentication hangs for minutes, then recovers on its own. Not memory (5.3 GB free, no swap, load 1.4 at the time); twice right after restarting the brain + Studio | next time: `journalctl -u ssh` on the Pi's own console; check `UseDNS` / reverse DNS for the Jetson's address |
| E14 | **Owner sees "some issue" in RViz -- not yet described** | checked 2026-10-04: every enabled display has a publisher and RViz is subscribed; blocker views carry all colours; no RViz errors but one, `glsl120/indexed_8bit_image` "active samplers ... same texture image unit", present in every RViz log since before that day (Intel Iris Plus, Mesa, GL 4.6). Remote screenshots are blocked by GNOME and an xwd grab garbles the 3D view, so the screen itself is unseen. A black square the size of the 8 x 8 m costmap showed in that grab | ask the owner what it looks like; first test: untick "Walls + body margin" -- if a black square goes, its clear cells draw black on this laptop |
| E12 | **Saved places are gone** | the power cut restarted odom; places saved before it are in the old frame | re-save the ones wanted |
| E15 | **"Go near the door" took 4.9 min; the yellow nav2 plan appeared only in the last 30 s** | Studio thread `01a10695`, 16:40:01-16:44:56 IST. The first `look()` VLM box was the whole door height, `[112,0,266,324]`. `pixel_to_goal` chose 1.76 m, but its depth slabs were at 2.1-2.3 m with clutter at 1.51 m, so the goal landed in front of the door among furniture at odom (-1.08, 0.65). `reach` moved it 25 cm to a free spot, but the only way in was a 31.6 cm gap (the rover needs 42 cm). nav2 said "no valid path found" every ~1.5 s for 40 s (so no `/plan`, no yellow line); then gap-pass "too tight", goal_exec "obstacle 0.42 m into the leg", nav2 no path again, a pass attempt that stalled, a 4th nav2 try. Cancelled at 241 s. A fresh look picked another part of the door, 2.48 m away and **confirmed by the LiDAR** (5 of 9 scan points). The plan drew at once and the rover arrived in 32 s, 3.3 cm off | (1) a large VLM box (door, wall) should take the LiDAR surface like the second query did, not the nearest depth slab; (2) when nav2's first plan fails "no valid path", try another pixel or look again within seconds, instead of spending 241 s on retries |

## 2026-10-04 — a Studio conversation with the robot, reviewed against the logs

Studio thread `01a104f3`, 08:56-09:11, rover on the fused pose; checked against
Jetson `reach` / `goal_exec` / `pixel_to_goal` logs and Pi 5 `/tmp/langgraph_dev.log`.
Fixes are in the Pi 5 brain (`~/ros2_ws`) unless marked Jetson.
**Fixed 2026-10-04:** S1-S5 in Pi 5 `a362e8a` (492 tests pass; deployed, brain +
Studio restarted); S7 + S10 in `3b15b5e` (reach: way back along its trail, look
first, stop after 3 identical failures; live-tested, see the commit). The `rover
wheels` check now also checks the ESP32 is subscribed to `/cmd_vel` (`2c29381`).
**Ghost marks fixed 2026-10-04 (owner's choice: the LiDAR rule):**
`phase2/nodes/ghost_clear.py` sits between nvblox and nav2 / goal_exec / reach
(`/ghost_clear/grid`): a camera mark the LiDAR also hit is tall (a person);
when the LiDAR sees through it, it is erased; marks under the rover are
erased; a mark only the camera saw (a shoe) is never erased by the LiDAR.
Floor-tested with the owner: legs left behind the rover after a turn were
erased (no LiDAR there now); a box under the LiDAR beam stayed kept (45 marks,
0 erased) with the camera turned 104 deg away. 15% of a core. Watch it:
`phase2/tools/ghost_watch.py`; RViz: `/ghost_clear/erased`; off:
`ROVER_GHOST_CLEAR=off`. Not yet seen: a few wall cells at 2+ m erased (a
slanting wall's return from beyond them) -- watch for low things at walls.
**Still open:** S6 (a small turn stalls beside furniture) needs a floor test; S8
(user corrections) not started; the ghost marks themselves (below) -- the way
back routes around them only when a look clears them. On 2026-10-04 the rover
was boxed in by marks under its own outline (LiDAR nearest 31 cm) and only a
map restart (`./rover map`) freed it. Clearing them for real is a choice for
the owner: nvblox decay off is deliberate (launch file, "DO NOT LET THE MAP
FORGET"), so the options are a slow decay, nvblox's dynamic mode, or clearing
marks the rover's own body has driven over.

| # | what the robot did | what really happened (evidence) | fix |
|---|---|---|---|
| S1 | "Where are you now?" -> "about 1.5 m from the door" | it had **arrived** at 09:00:30, 4.4 cm (reach). Studio log: `nav done ... listener=NONE` -- the arrival was thrown away. The Dad drive's failure (09:16, "gave up after 4 attempts") was thrown away too, so "say good morning" could never run | Studio registers the nav-done + system-turn callbacks like agent_node (issue 8), or Studio refuses to drive and says so |
| S2 | "check your location again" -> `move_robot S` (a STOP), then "I am positioned near the door" | no tool reports where the robot is or whether a drive is running; `get_robot_status` only says camera + body. It guessed, and stopped the robot to do it | `get_robot_status` adds pose, the active drive (label, distance left) and the last drive's result |
| S3 | "send me the picture" -> "I have sent a photo to Rakesh" | the tool **failed**: `I don't know 'Boss' on Telegram`. Nothing was sent | "Boss"/"me" resolves to the owner; the failure text says "NOT sent -- tell the user" (as scan_surroundings does) |
| S4 | "go near the person" -> after **286 s**: "couldn't measure the distance ... too close, too far, or reflective" | it **had driven 2.5 m** to her (reached 09:04:53, 7.5 cm). The arrival photo then lost its depth (`no_depth_near_stamp: 7849 ms`, `moved_before_depth`), the VLM took 45 s, the hold expired; the retry photo no longer showed her (she moved). The reply hid the drive and blamed reflectivity | the reply says what happened: "I got close; she moved out of view." Re-ground on the fresh photo without a second 45 s locate when the robot is still. Jetson: why depth stopped for 7.8 s after the drive (open issue 1) |
| S5 | "go near my Dad" -> "I have located your father" | the robot cannot recognise people. Photo recall picked the floral-dress woman's spot (same pixel 56,164, same 2.74 m as 09:03), then the search took the first man it saw, 4.1 m away. It passed "the man who is my Dad" to the VLM verbatim | person identity: ask the user what Dad looks like (or say it cannot tell who is who) before driving. A tool rule, not a prompt hope |
| S6 | door: first attempt "goal_exec stalled: turn stuck -26.8 deg" | Jetson: a -27 deg turn beside the cot moved 1.5 deg, twice; escape timed out, pass failed; reach gave up at 41 s. The search from where it stood then reached the door in 14 s | Jetson: why a small turn stalls next to furniture (swing check vs. deadband) -- reproduce beside the cot |
| S7 | Dad drive: 316 s of "no path: planner found none" | the goal 4.1 m away (through the doorway) had no plan, 4 attempts; nobody was told (S1) | after S1, the failure reaches the user; reach could say "no path" after 2 tries, not 4 |
| S8 | the woven cot was called "a decorative screen", and the correction was only acknowledged | VLM limit (issue 3); nothing stores a user's correction | optional: a user correction goes into the photo log as a label |
| S9 | "what are you looking at" while in `navigate` -> tried `scan_surroundings` | the backstop caught it and handed to `local_agent` (works as designed) | -- |

**S10 — the Dad drive went the wrong way and stayed stuck (Jetson).** The goal
(the brother, 4.1 m, (-0.73, -0.68) odom, in the other room) was back through the
doorway the rover had just driven in by. nav2 sent it 2 m **sideways** instead
((3.46,-0.41) -> (2.20,-2.49) in 17 s), into the pocket behind the cot, and from
there every plan failed for 5 min (4 identical retries, backup/wait each time).
Cause, measured on the live maps afterwards: the LiDAR SLAM map shows the
corridor it drove in by **free**; nvblox's grid has **specks** in it (12 blobs of
1-3 cells + larger ones, 2.5 cm cells, `/nvblox_node/static_occupancy_grid`),
each 0.04-0.2 m from the route. Rebuilt from nvblox alone with nav2's inflation,
the corridor costs match the real global costmap (99 = blocked wherever a speck
is within ~0.2 m). So the way back was closed by depth leftovers -- likely the
people who walked through the doorway (nvblox does not decay, issue 5) -- and
nothing in reach/nav2 re-checks them: the camera faced away, so they never
cleared. reach also never tries the path it **just drove**, which it knows is
passable. Pictures: `docs/img/2026-10-04_dad_drive_*.png`.
Fixes: (a) on "no path", reach turns the camera toward the blocked direction so
nvblox re-observes and clears it, then replans; (b) a breadcrumb fallback --
retrace the rover's own recent track (checked against the live LiDAR, not
nvblox) toward the goal; (c) two identical "no path" from the same spot ->
stop and report, not 4; (d) maybe: drop nvblox specks the LiDAR does not
confirm *above the LiDAR's height only* -- but chair legs are 1-3 cells too,
so not without a floor test.

Slow turns: "move towards the door" 88 s, "go near the person" 286 s, "go near Dad" 83 s
before its drive started -- most of it VLM round trips (issue 3).

## 2026-10-03 (late) — Pi 5 ssh/IDE, ports, real-rover bring-up

Focus is the **real rover only**; the twin is parked (owner reviews it later).

| item | finding | status |
|---|---|---|
| ssh to the Pi 5 | 192.168.1.16 answers (MAC 2c:cf:67:24:d6:bf, hostname `rakhi24-desktop`, `~/ros2_ws` there); sshd on :22, all interfaces. 192.168.1.22 is **no host at all** (no ARP reply, no route). Nothing on the Jetson, Pi 5 or laptop (.17) mentions .22, so it is a saved connection in the **IDE client** (JetBrains Gateway/PyCharm on the machine at .6, which does log into the Pi 5 by password) | **owner: edit/delete the saved .22 connection in Gateway; recreate it as rakhi24@192.168.1.16, port 22, project `/home/rakhi24/ros2_ws`** |
| PyCharm can't open `~/ros2_ws` on the Pi 5 | `.idea/workspace.xml` was corrupt (a truncated save left `mponent>\n</project>` after the end, "junk after document element", line 257); PyCharm logged "Save blocked". It also has only the old 252 backend; the client is 262, so Gateway will download 262 the first time | fixed: moved aside to `.idea/workspace.xml.corrupt-2026-10-03` (per-user IDE state, untracked; PyCharm rebuilds it) |
| ports, Pi 5 | TCP: 22 (ssh), 8091 (teleop), 127.0.0.1:8090 (agent_node), 127.0.0.1:2024 (Studio). UDP: 8888 micro-ROS, 7000-7004 + 74xx DDS, 11811 discovery server. No strays, no duplicates | clean |
| ports, Jetson | TCP: 22, PyCharm remote-dev on 127.0.0.1 only (5990/63342/...), 5201 iperf3 server (left from the switch test), 6566 (saned) | clean. iperf3 is harmless; stop it if unwanted |
| Mac mini LLM/VLM | Was not on the rover's network (the owner moved it). Now `singireddys-mac-mini.local` = **192.168.1.5** from the Pi 5; `/health` ok; text reply 0.8 s; a live D555 frame described correctly (office chair + white chair, tiled floor) in 4.3 s. (.6 = owner's MacBook Air; .7:8080 is an unrelated web server) | **fixed**. The brain calls it by name, so no config change |
| bring-up | `./rover up` exit 0: IR 30 Hz, depth 28.6, VO 28.4, gyro 203, lidar 10, lidar odom 10, `/odom` 20 (LiDAR-anchored, sd 0.6-0.7 cm), cuVSLAM 44 landmarks, slam map 0.5, nvblox 4.9, costmaps 0.9/1.7, colour for Pi 5 4.6, wheel_state 20-21.5; teleop AUTO; RViz on the laptop (.17) | up |
| twin on the laptop | `home_real` world was still running from 18:42 (domain 42, so it never touched the real wheels, but it was using laptop CPU next to RViz) | stopped with `./mitra down`. Twin brain (Pi 5) and twin Jetson stack were already down. Restart all: `fleet.sh twin up` on the Pi 5 |
| Jetson `/etc/hosts` | `192.168.2.10 rakhi24-desktop.local` ("ethernet via JioAirFiber switch") was left over, so on the Jetson the Pi 5's name went to a dead address; the container copies it at creation | fixed: now `192.168.1.16` on the host (backup `/etc/hosts.bak-2026-10-03`) and in the running container |
| "go near the door" from Studio: "I can't find the door" | the brain looked once (`snapshot_expired`), then both search turns (-45, +94 deg) **stalled at 0 deg rotation**; 0.61 m+ clear all round, so not boxed. Nudge: wheel peaks 0.060/0.068 vs 0.090/0.086 at 20:43 for the same 0.05 m/s: **battery sagging**; turns (skid scrub) fail before straight driving. The rover had also been moved 1.1 m / 112 deg after arriving at the door (pose healthy, same origin) | **owner: charge before more motion tests.** Code: a stalled search turn should say "my turn didn't move, battery may be low", not "can't find it" (D3) |
| two Studio servers | a second `langgraph dev` (19:28) could not get :2024 but kept its workers and a second `/studio_bridge` | killed the orphan; one bridge now |
| Studio + agent_node both up | both publish `/cmd_vel` and poll Telegram | known (see STARTUP.md): stop Studio before driving from Telegram |

## 2026-10-03 (evening) — wired network: the Waveshare switch and the D555 (PARKED, owner tests later)

Goal: D555 + Jetson + Pi 5 on one gigabit switch (Waveshare Industrial 5P, SKU TIFPS0683, RTL8367N),
no WiFi between Jetson and Pi 5. **Today the camera is back on its direct cable (works) and the Pi 5
talks to the Jetson over WiFi.** Jetson enP8p1s0 = 192.168.11.70 only, MTU 9000, as before.

| measured | result |
|---|---|
| D555 network config (`~/rs_build/rs-dds-config`, read-only) | MTU **9000**; `--mtu 1500` refused: "Camera FW supports only MTU 9000" (FW 7.56.19919.4144). Nothing written to the camera |
| D555 streaming, direct cable | 4764 of 5000 packets are **9010-byte** frames; 659 Mbit/s; IR 30 Hz, depth 30 Hz |
| D555 through the switch (2 ports, same cable + injector, full 2-3 min boot) | only <= 700-byte packets arrive (discovery, IMU); no jumbo frame ever; driver "zeroed intrinsics" / "No RealSense devices" / emitter "control channel not ready" |
| switch port lights | Jetson port yellow + orange, camera port yellow only (camera leg possibly negotiated < 1000 Mb/s) |
| Jetson <-> Pi 5 through the switch (DDS, 200 kB messages) | 160/160 each way at MTU 1500; 0/160 when the Jetson port was 9000 (the Pi 5 is 1500) |

Waveshare's wiki claims "forwarding 9216 bytes huge data packet". Two explanations remain, both on
the switch side: (1) the board only forwards ~1536-byte frames, or (2) the camera's port negotiates
100 Mb/s with this switch (a 660 Mbit/s stream cannot fit). The clean test: Jetson <-> Pi 5 through the
switch with both at MTU 9000 (`sudo ip link set eth0 mtu 9000` on the Pi 5, temporary) and a ping-size
sweep. Options: a USB 3 gigabit adapter on the Jetson (camera direct, switch on the adapter) --
recommended; D555 FW 7.58.38448.10612 (librealsense 2.58.2's, "enables configuration of MTU size and
delay") then `rs-dds-config --mtu 1500 --transmission-delay N`, only helps under (1); or a switch with
real 9K jumbo. Traps found on the way (memory note): the D555 refuses pings > 1472 bytes although its
data is jumbo; it takes ~2 min to boot; plain `rs-enumerate-devices` in the container has no DDS (use
`LD_LIBRARY_PATH=/root/librealsense/install/lib`).

**2026-10-04 retest with the owner -- the switch drops every frame over ~1.5 kB.** Both cables were proven
first, each on its own direct to the camera: the RealSense cable, and an unbranded Cat6+ that sent 45,105
x 8968-byte payloads in 5 s, IR/depth 30 Hz, 0 errors, and real IR/colour/depth images (checked by eye).
Then Jetson -> Cat6+ -> switch -> RealSense cable -> D555:

| run | link (Jetson side) | largest frame from camera | topics |
|---|---|---|---|
| ports A, camera not power-cycled | 1000 Mb/s, 0 errors | 1384-byte payload; **0 jumbo** in 10 s | IMU 200.9 Hz; IR1, IR2, depth, colour, aligned: **no data** |
| ports B (both cables moved), D555 PoE-cycled | 1000 Mb/s, 0 errors | 702-byte frame; **0 jumbo** in 10 s | IMU 201.8 Hz; every image topic **no data**; driver "notification sample(s) lost" x34, "control channel not ready" |

Two ports and two runs, proven cables, and the same result: small packets pass and nothing large ever
does. That is explanation (1) above. Explanation (2), a 100 Mb/s camera leg, would still let *some*
9010-byte frames through, and none ever arrived. Return reason: the switch does not forward jumbo
frames, which its wiki advertises (9216 bytes).

**The Pi 5 cannot be the test partner for a ping sweep.** Its eth0 (macb, kernel 6.8.0-1065-raspi)
reports maxmtu 10222 and takes MTU 9000 (only with the link down: "Device or resource busy"
otherwise), but on a *direct* cable to the Jetson nothing over 1500 crossed in either direction, not even
1600 bytes. Its rx counters didn't move and tcpdump on the Jetson saw only 1514-byte frames. Set back to
1500 the same day. A ping sweep needs a laptop whose jumbo ping passes on a direct cable first.
The procedure for that, from the Mac mini: [WAVESHARE_SWITCH_TEST.md](WAVESHARE_SWITCH_TEST.md).

**2026-10-07 retest, direct cable Jetson `enP8p1s0` <-> Pi 5 `eth0` -- jumbo frames DO work; the note above is overturned.**
Pi 5 set with `sudo ip link set eth0 down; ... mtu 9000; ... up` (temporary, run over WiFi SSH); Jetson given a
temporary 192.168.2.70/24 on `enP8p1s0` (Pi 5 eth0 is 192.168.2.10/24), removed afterwards. Both links 1000 Mb/s.
Ping with DF set: payloads 1472/1600/2000/4000/8000/8972 all 5/5 Jetson->Pi, 1472/1600/4000/8972 all 5/5 Pi->Jetson,
RTT 0.6 ms at 8972. Then 2000 x 8972-byte pings at 2 ms spacing: 2000/2000, 0 errors/drops; the Ethernet port carried
18 MB (2110 packets, ~8.6 kB each) while WiFi sent 107 kB, and WiFi's MTU is 1500 so DF jumbo cannot route there.
Before the Pi 5 was at 9000, everything over 1472 failed, so the earlier "nothing over 1500 crossed" was most likely
the MTU not actually applied (NetworkManager's `netplan-eth0` has mtu `auto`; check `ip link show eth0` after setting).
**So the Pi 5 IS a valid jumbo test partner** for the switch sweep (WAVESHARE_SWITCH_TEST.md) -- no laptop needed.
The Pi 5's MTU 9000 lasts until reboot.

**2026-10-07, same pair through the Waveshare switch -- the switch has a hard 1500-byte limit. Final.**
Jetson and Pi 5 each on a switch port, both MTU 9000, both 1000 Mb/s, same addresses. DF ping payloads Jetson->Pi:
1472 passes (3/3); 1480, 1500, 1518, 1550, 1600, 2000, 4000, 8972 all 0/3. Pi->Jetson: 1472 passes; 1500, 1600, 8972
all 0/3. The owner's own run: 1472 10/10, 8972 0/10. The cutoff is exactly a standard 1518-byte Ethernet frame
(1500-byte IP packet), so the switch does not do jumbo at all, whatever the wiki says. Every part other than the switch
is proven (both cables, D555, Jetson, Pi 5). **Return/replace the switch; any replacement must be tested with this
same ping (`ping -M do -s 8972`) before the camera goes on it.**

## 2026-10-03 (night) — the Mitra twin (simulator) and what it found

A laptop twin of this rover: github.com/Rakeshreddysr2401/mitra_sim
(private). The laptop is only the WORLD (measured body, lidar, depth camera,
gyro, the wheels' real slide); THIS code is the brain, then run as a second
nav-only instance in the rover container on ROS domain 42, beside the real
stack on domain 0 (since 2026-10-11 the whole stack: `TWIN=1 ./rover up`). Every run is judged on
the true pose against the true obstacles. Results: mitra_sim docs/STATUS.md.

| found by the twin | status |
|---|---|
| goals > ~4 m away failed at once: the global costmap is an 8 x 8 m rolling window ("outside bounds") | fixed 302de1b: reach drives legs of <= 3.2 m |
| turn-and-shuffle came 0.2 cm from a box while the pivot was unlearned | fixed 302de1b: goal_exec +3 cm until learned |
| things under 3 cm (slippers, a laptop) are invisible to nvblox and the lidar: driven over | **open** -- a 1 cm slice sees them but floor noise then boxes the rover in (rejected); next: 2 cm, or a close-range floor sensor |
| gap passes 1-4 cm from the chair leg (S-gap 45/50/60) | **open** -- MPPI margin 4 cm froze it (rejected); next: speed / cost near obstacles |

The real rover's reach / goal_exec load 302de1b at the next `./rover nav`.

## 2026-10-03 — close quarters (narrow-gap tests with the owner)

Test: a 45 cm gap, spray bottle one side, a ~7 cm chair leg the other (an
S-gap); the rover touched the leg in all three runs. Root causes and fixes,
all committed on `rover-v1.1.7` (33fd348, 146361c, 15029cd, 11ee9b7 + the
review commit after it):

| what | fix | status |
|---|---|---|
| Nav2 Spin swung the outline blind in the gap | Spin out of both BT recoveries | in |
| reach's pass refused since 09-27 (sent 1 cm, goal_exec needs 2) | `PASS_MARGIN = 0.02` | in |
| Turn smear: objects mapped again 3-10 cm beside themselves | `phase2/nodes/turn_gate.py` + `depth_gate.py`: depth to nvblox only after 0.8 s with no turn; depth memory places frames at their stamp | measured (central-view smear 17 -> 5 cells at 6-10 cm) |
| Chair leg barely in the map | nvblox slice 5 -> 3 cm | measured (leg ~6 cells -> tip + edge) |
| Planner routed with 0.8 cm to the leg | global `footprint_padding` 0.04 | plan-checked; 45 cm gap = no path |
| Live depth reads mid-air points in front of white walls (emitter off for cuVSLAM) | exact moves use `phase3/nodes/fused_obstacles.py` (nvblox fused map); `ROVER_OBSTACLES=depth` reverts | live turns OK; tests `phase3/tools/test_obstacle_sources.py` |
| reach drove into the gap mouth / looped escape + nav2 | on no path or a jam: look, straight pass first (1.8 / 1.3 / 0.9 m) | floor run at 60 cm next |

**Open:**
- A 45 cm gap is below what can be promised: the fused map is 2.5 cm cells,
  the pass needs 38 + 2 x 2 cm, and it reads 41 cm. sim_gap_pass agrees
  (46 cm is the boundary). For 45: a sharper near view -- tilt the D555 down
  15-20 deg, or a low front webcam + a floor/not-floor model (owner has a
  Logitech webcam), or learned stereo (Isaac ROS ESS; not in the image).
- footprint_clear (global) clears 2 cm past the now 4 cm padding: the
  planner does not see anything within ~6 cm of the rover's outline.
- After moving obstacles, restart the map layer (`./rover map`): the static
  map has no decay, so the old positions stay until seen free.
- depth_gate costs ~17% of one core (Python relay of raw frames); a C++
  relay if CPU gets tight.


## 2026-10-02 (Pi 5 brain, floor-tested with the owner)

The robot's memory is now its PHOTOS (Pi 5 `tools/photo_recall.py`, `ask_photos`): the VLM reads every logged photo, picks one and a box, and pixel_to_goal places it with THAT photo's held depth + pose -- the background object survey is off. "go near X" asks the photos first; on arrival code takes one photo and checks X is really there; the drive tools carry an errand (`then`: "...and tell me what is on it") to the arrival report. Teleop: the page re-reads the mode every 2 s and logs every switch (81caf56) -- and the Pi 5 was still running the 08-22 copy, so 9d780b1 (MANUAL cancels reach + goal_exec) was **deployed only now**.

One page to come back to. Details and measurements live in the linked
documents; this is the list.

## Done 2026-09-26 (floor-tested with the owner)

| what | result | where |
|---|---|---|
| B1: the brain moves through the Jetson's exact controller | `move_robot` turns 6/6 within 2°; `F:30,L:90,F:30` legs end 0.8 / 0.5 cm; navigation goes through `/reach` | INTELLIGENCE_PLAN.md §5 |
| B2: coordinates from the moment of the photo, on the VLM's box | one bottle located from 3 poses: all within 0.7 cm (was 65 cm) | INTELLIGENCE_PLAN.md §5 |
| B3: object memory + the checker | found a bottle 1.45 m away, arrived 1.4 cm; backed off and turned away, "go near the bottle" turned back to it, "still where I saw it", arrived 1.0 cm | INTELLIGENCE_PLAN.md §5 |
| goal_exec turn fix | retries judged while coasting + arrival-time jitter; reproduced in the simulator, fixed | commit d313f5b |
| Idle depth subscriptions dropped | Jetson aligned depth 22 → 28 Hz, stalls > 0.5 s in 45 s: 3 → 0 (one sample) | commit 38a4c52 |
| Studio stale run queue | cleared; orphaned worker processes were writing it back | memory note |
| Lingering-obstacle checker | `phase3/tools/ghost_check.py` | OPERATIONS.md §5 |

## Fixed 2026-09-28 — code review of both repos (offline; NOT yet run on the rover)

Each needs one look on the floor after deploy; the "check" column says what.

| what | fix | check on the rover |
|---|---|---|
| **Object memory ids collided** (Pi 5) | the id was the sighting time in ms, and the survey stores every object of a photo with that photo's time: all ~5 shared one id, so forgetting one forgot the whole photo, and approach's "has it moved?" compared an entry with itself (also why 2 tests were flaky). Ids are now unique; old files are repaired on load | "go near X" after a survey; `/status` → `objects_remembered` |
| `reach_and_wait` could cancel its own goal (Pi 5) | set the new goal's key BEFORE cancelling the background drive, as `start_nav_to_pose` already did; the old order let the old worker send `/reach/cancel` after the new goal | "go to the chair" while a drive is running |
| Late pixel replies wiped a waiting one (Pi 5) | the reply cache dropped ALL entries at 32; now drops the oldest | — |
| `#9` again, in 8 more places | `set -e` + a probe of something that is down (`curl` 7, `grep`/`ls`/`tail` finding nothing) ended the script silently: `./rover goto/navto/drive/pass` with teleop down, `./rover camera` emitter retry, `./rover view`, `./rover grade`, `./rover status` (no nav2 restarts yet → no health report), and the Pi 5's `fleet.sh check` when the brain/LLM/teleop was down — the moment it exists for. New `refuse_if_manual` (uses `$PI5_HOST`, not a hard-coded IP) | `./rover status` on a fresh nav2; `fleet.sh check` with the brain stopped |
| `fleet.sh check` said "LLM down" when a token is set | sends `LANGROBO_API_TOKEN` from `.env`; a 401 is its own FAIL | — |
| Depth gap + TF lag dropped the photo (Jetson) | pixel_to_goal now also waits for the photo's camera pose when TF lags the stamp (both come from the same CPU load), then the still-camera rule of c4a5ee5 | `grep 'after a depth gap' /tmp/pixel_to_goal.log` |
| A gyro stall silenced `/fusion/status` too (Jetson, #2) | status keeps coming from the LiDAR callback with `gyro_alive: false`; **goal_exec refuses to move on it** ("gyro stalled: the pose is frozen") — it used to refuse only because status went silent; `health.py` says so | `./rover status` during the next stall: pose ✗ "the gyro has stopped" |

From the floor-test record (TODO / commit history; the robot's own logs were not reachable):

| what | fix | check on the rover |
|---|---|---|
| **The photo survey used a slot the documented server does not have** (Pi 5) | every doc said `--parallel 3` (slots 0-2); the survey pins slot 3, so on such a server each survey call failed, counted only as an error, and object memory stayed empty. **2026-09-29:** slot 3 is the vision-TOOL slot for every one-shot photo prompt (search views, locate, survey) -- `_vlm_locate` used to run on local_agent's slot 1 and overwrite the vision conversation's cached photos up to 8 times per search; on a smaller server the survey is off rather than evicting an agent; docs and `fleet.sh check` say `--parallel 4 --swa-full` | Pi 5 boot log: "vision-tool slot 3"; `/status` → `photo_survey.enabled` true, `errors` 0 |
| **MANUAL flipped mid-move did not stop the brain** (Pi 5; 2026-09-27 a "look around" turn went ~120° across a MANUAL→AUTO flip) | the switch was checked only when a tool started; a scan/search kept turning against teleop's zeros and a reach retried for minutes. agent_node now watches the switch (2 Hz) and on the edge into MANUAL cancels the drive and interrupts the tool, as a new utterance does | "look around", flip MANUAL after the first turn: it stops and says so |

Second review, 2026-09-29:

| what | fix | check on the rover |
|---|---|---|
| **Any Telegram member could drive the robot** (Pi 5, security) | `CAP_MOVE` was defined and never checked; family and guests could send it anywhere. Every motion tool now refuses a sender without it; "stop" stays open to all. Tool schemas unchanged (the KV cache is untouched) | from a family account: "go forward" is refused, "stop" works |
| **MANUAL did not cancel reach or goal_exec** (teleop) | it cancelled nav2 goals only; goal_exec kept commanding against teleop's zeros (the rover jerks) and reach retried for minutes. MANUAL now also publishes `/reach/cancel` and `/goal_exec/cancel`. **Redeploy:** copy `phase1/teleop/teleop_web.py` to `~/langrobo_teleop/` on the Pi 5, then `pkill -f teleop_web.py` (phase1/teleop/README.md) | `./rover goto 0.5 0` then flip MANUAL: stops at once **Deployed 2026-10-02** -- it had never reached the Pi 5; verified: teleop publishes /reach/cancel and /goal_exec/cancel. |
| Saved places lost on a power cut mid-save (Pi 5) | `locations.json` is written atomically | — |

Checked and fine: the ESP32's 500 ms /cmd_vel watchdog; goal_exec and gap-pass
simulator suites (both SUITE PASS).

New: `GET :8090/status` on the Pi 5 carries `photo_survey` (photos, objects,
errors, queue) and `objects_remembered`; `fleet.sh check` prints them.

## Open — most damaging first

| # | issue | evidence | next step |
|---|---|---|---|
| 1 | **The Jetson is CPU-bound, and the camera pays for it** | every core 76-88%; the D555 driver logs "callback took too long"; depth stopped for up to 9 s (in the frames' own stamps); `/odom` stamped 50 ms apart arrived 0.6-441 ms apart | a longer measurement, then the heavy Python nodes: fusion2 ~40%, lidar_odom ~38%, gyro ~29%, goal_exec / reach ~27% idle (likely TF listeners at 66 Hz), pixel_to_goal ~25%, image_bridge ~21% **2026-10-02:** idle py-spy showed the Python nodes spending 77-86% of their time in rclpy's stock executor; EventsExecutor cut pixel_to_goal 24.7->11.0%, goal_exec 26.4->8.1%, image_bridge 21.1->17.5%, gyro 28.7->15.7%, reach 24.3->8.4% (each floor-tested); Jetson idle 27.0->37.8%. **fusion2 + lidar_odom are changed but load only at the next ./rover up: grade them then** (turns within 2 deg, F:30 ending ~1 cm, /fusion/status healthy; ROVER_EXECUTOR=single reverts). Also: nvpmodel is 15W (CPU 1.5 of 1.73 GHz) -- a higher mode is the owner's call (power supply). |
| 2 | **The gyro rides the camera's link** (LOCALIZATION_GAPS G12) | the same link stalled for 9 s; fusion2 publishes the pose from the gyro callback, so a camera stall freezes the pose; since 2026-09-28 the status says so (`gyro_alive: false`) and goal_exec refuses | an IMU on the ESP32 or the Jetson; or fusion2 predicting on wheels + LiDAR while the gyro is out (an estimator change: grade it offline on a recording with a gyro gap first) |
| 3 | **The VLM is slow and loose** | Gemma 4 12B on the Mac: ~9 tok/s writing; y off by up to 45 px (boxes + pixel_to_goal's padded slab work around it); sensitive to wording ("spray can" missed where "air freshener spray can" was found) and misses flat things (a keyboard). Photo questions now ~5 s warm (photos cached on slot 3). A false sighting seen once (a toy car taken for something by a backpack) -- caught by the Pi 5 arrival check | the Mac: `--swa-full` + speculative decoding (needs ssh to the Mac); a better model later |
| 4 | ~~The checker's "bottle gone" path is only unit-tested~~ **superseded 2026-10-02** | the photo-based approach + the arrival check: floor-tested "it has moved about 0.6 m", "can't see the toy car in front of me" (a wrong sighting), and a full search for an absent object | -- |
| 5 | ~~Obstacles linger in the RViz costmap~~ **fixed 2026-10-04 by `ghost_clear`** (0efdf58) | reproduced at last (S10): people's legs closed a doorway and boxed the rover in. Now a camera mark the LiDAR also saw, and later sees through, is erased; low things stay (floor-tested) | what remains is a LiDAR-layer mark in the SHADOW of something nearer (no beam reaches it): `ghost_check.py` + `ghost_watch.py` (OPERATIONS.md §5) |
| 6 | Turns land slightly short; slides are large | all within 2° but ~1.3° short; a 180° turn slid 34 cm | tighten the tolerance once the coast is modelled; the slide is the pivot, which moves with the battery |
| 7 | ~~Object memory~~ **replaced 2026-10-02 by the photo log** | the Pi 5 no longer keeps "object at x,y" (owner's call); photos are asked directly, 24 kept = SNAPSHOTS here | pixel_to_goal's 24 held snapshots are now the memory's depth: keep SNAPSHOTS >= the Pi 5's photos.MAX_PHOTOS |
| 8 | Studio and the Telegram brain run the same graph | both can drive; a restart that kills only the parent leaves orphaned workers. **Since 2026-10-04 Studio's own drives report back into its thread** (Pi 5 a362e8a) | one brain at a time for DRIVING; restart kills the whole process tree. (Studio takes no voice/Telegram input — only agent_node polls — so it cannot steal a request.) Pi 5 skills are now `/robot-start` and `/robot-stop` |
| 9 | ~~`./rover up` exited 7 right after starting Studio~~ **fixed 2026-09-27** | curl's exit 7 (Studio not listening yet) inside a `$(...)` under `set -e` ended the script; two sibling checks had the same trap | `|| true` on all three; the laptop RViz step runs again |
| 10 | Seen once, not reproduced | a captured frame PIL could not open; a Pi 5 ssh drop mid-run | watch |
| 11 | Localization leftovers | depth time offset unmeasured (G8); wheel ticks unstamped (G6); Pi 5 clock ahead of the Jetson (G9) | LOCALIZATION_GAPS.md |
| 12 | Power system | the turn slide depends on the battery; charge before motion tests | ROVER_BUILD_PLAN.md §4.5 |

## Ahead in the plan

| step | what | INTELLIGENCE_PLAN.md |
|---|---|---|
| B4 | fast eyes: a detector on the GPU (TensorRT), feeding object memory | §4 |
| B5 | smart search: overlapping exact turns, one VLM call lists everything | §4 |
| B6 | learning loop: episode log, weekly detector fine-tune on the owner's own objects | §1.4, §4 |

## Suggested order

0. Deploy both repos and walk the "check on the rover" column of the
   2026-09-28 table (≈10 minutes).
1. #4 — two minutes at the rover.
2. #1 + #2 together — CPU load and the camera-borne gyro are behind most of
   the flakiness, and B4 needs a healthy Jetson.
3. B4, then B5.

## Housekeeping

- The Pi 5 sudo password was typed into a chat session on 2026-09-26: change it.
- 2026-10-03 tidy of all repos and machines (SYSTEM_OVERVIEW.md §5, C1–C13): done. The
  leftover hand copy of the twin's Jetson files was deleted 2026-10-18.
