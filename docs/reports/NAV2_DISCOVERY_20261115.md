# nav2 does not start with the full stack — what we know (2026-11-15 evening)

Read this first when you are back. Short version: **every sensor and the Pi 5 link work; nav2 (the
part that plans and drives to a goal) never becomes active.** It was restarted 11+ times in 30 minutes.
Nothing is broken in hardware. The cause is in how the ROS nodes find each other (DDS discovery).

## 1. What works

`./rover up` after your power cycle of the Pi 5 and the D555 — every gate OK:

| check | value | wanted |
|---|---|---|
| camera IR / depth | 30.0 / 30.8 Hz | 15 / 10 |
| lidar scan | 10.0 Hz | 8 |
| visual odometry / gyro | 30 / 200 Hz | 10 / 50 |
| fused odometry | 20.0 Hz | 15 |
| lidar map / camera map | 0.5 / 4.8 Hz | 0.3 / 1 |
| camera picture received on the Pi 5 | 4.2 Hz | 2 |
| wheels (`/wheel_state`, from the Pi 5) | 19.7 Hz | 15 |
| Pi 5 clock vs Jetson clock | +37 ms | within 100 |

Both repos: all tests pass (Jetson 31 test files, Pi 5 895 tests).

## 2. What does not work

nav2 never prints "Managed nodes are active". The bring-up gates stay green because they only look
at the costmap rates, so `./rover up` did not show the failure.

What nav2's log (`./rover logs nav`) says, by start attempt:

- 6 of the first 7 starts: `"follow_path"` / `"compute_path_to_pose" action server not available after
  waiting for 5.00s`. That is nav2's own planner and controller, started seconds earlier in the same
  launch, not being found.
- After I raised that wait to 60 s: once it still failed after 60 s; twice a costmap got no
  `odom -> base_link` transform within 60 s; the rest ran past the supervisor's 150 s limit.

## 3. What I measured (scripts in `tools/net/`, they only read packets)

1. **Three quarters of all local network packets are discovery chatter.** On the Jetson: 26,000
   packets a second; about 20,000 of them are nodes asking each other again and again "what topics
   and message types do you have?" (14,200/s type lookups + 5,600/s endpoint lists). It never
   settles. Real data is only 3,600 packets a second.
2. **This is not normal.** 14 plain test nodes on a separate test domain (77), same container image,
   same settings: zero such chatter once they have found each other.
3. **New nodes do not reliably find running ones.** A brand-new test node found `/tf`, `/scan` and
   `/map` in 1 second but did not find the camera's topics in 75 seconds. Another time a new node
   missed camera, lidar and maps for minutes.
4. **The "flood" to the Pi 5 from this afternoon is the same chatter**, not camera pictures. At good
   WiFi it is small (1,700 packets/s, 0.3 MB/s). On weak WiFi lost packets are asked for again, and it
   grows.
5. **The camera is on the same DDS domain as all the ROS nodes (domain 0).** The D555 is a network
   device that speaks DDS itself. Every one of the Jetson's ~36 nodes tries to exchange its topic
   list with the camera: 64 requests a second go to it, 8 answers a second come back. It never
   catches up. Only the camera driver needs to talk to the camera.
6. **No packets are lost in the kernel**, and the Jetson's load is 25-40 on 6 cores the whole time.

Points 1-4 and 6 are measured. That point 5 is the *cause* of 1-3 is my best explanation, **not yet
proven**.

## 4. What I propose (your decision)

**Step A — put the camera on its own DDS domain.** Then the ROS nodes and the camera never meet;
only the driver talks to it.

- Two changes: one setting stored in the camera (`rs-dds-config`, already in the container at
  `/opt/librealsense/bin/`), and the same number in `~/.realsense-config.json` (written by `./rover camera`).
- It can be set back to 0 the same way.
- It needs a camera restart, so probably one more PoE power cycle from you.
- Then measure the same three numbers again: chatter packets/s (`tools/net/lo_rtps.py`), how fast a
  new node finds everything (`tools/net/disc.py`), and whether nav2 starts.

**Step B — only if chatter remains:** switch off type lookup for the ROS nodes with a Fast DDS
settings file (`fastdds.type_propagation`, a documented option). Test on the twin first.

**Later, separate:** the camera pictures travel between programs on the Jetson as 84 MB/s of network
packets, because shared memory was switched off on 2026-10-18 for a real reason (pictures stopped
arriving). Doing shared memory properly (bigger memory area) could free a lot of CPU, but it needs
its own test on the twin.

## 5. What I changed tonight (Jetson, committed, not on GitHub)

| commit | what |
|---|---|
| 9bfe357, faff4ba | Watchdog: one long-lived probe instead of a new one every 30 s. My first version was NOT cheaper under the real load (36 % of a core); the second listens to fast topics 4 s of every 30 s: 10 %. It also takes one fresh look before it reports a topic as dead. |
| 5f2e8b2 | `./rover clock`: checks the Pi 5 and Jetson clocks agree (also inside `./rover up`). |
| 0a3e040 | nav2 waits 60 s (was 5) for its own planner/controller at start. Helps, does not cure. |
| faff4ba | `tools/net/`: the measuring scripts. |

Pi 5 (committed, not on GitHub): sonar node 7.6 % -> 1.5 % CPU (307bd53); one place for the DDS
settings, `kino_dds` in `scripts/ros_env.sh` (df254f4).

## 6. How the robot was left

- The stack is up (camera, lidar, pose, maps, Pi 5 link all running). The robot has not driven.
- **nav2 is stopped on purpose**: its supervisor restarted it every 2.5 minutes, which only adds load
  and leftover processes. To see the failure again: `./rover nav`, then `./rover logs nav`.
- Do not run `./rover camera` or `./rover up` again without a PoE cycle of the camera first.

## 7. Still waiting for you (unchanged)

1. The decision on step A above.
2. Pi 5: `scripts/servo/sts_bench.py`, `sts_chain.py`, `sts_range.py`, `sts_test.py` are empty files
   (0 bytes). They were never committed; they need to be written again.
3. Pi 5 power: "Undervoltage detected" again today. It needs the 5 V / 5 A supply.
4. Jetson disk 96 % full: `logs/bags` 69 GB, `logs/rl_demos` 23 GB — which to keep?
5. Push both repos to GitHub when you want.
