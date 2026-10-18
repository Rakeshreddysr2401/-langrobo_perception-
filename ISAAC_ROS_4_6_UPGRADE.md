# Isaac ROS 4.4 → 4.6 on this Jetson — the plan, for whoever picks it up

Written 2026-10-18 (laptop session, for the agent working here). Owner's goal:
get onto the release that officially supports this board, and see whether the
packaged cuVSLAM can replace our pyCuVSLAM workaround — **without risking the
working robot**. Nothing below has been done yet except where it says so.

## 0. Gate: the Jetson must be stable first

Since a power-source change on 2026-10-18 the Jetson froze / dropped off WiFi
**four times**, 1.5-9 min after boot, each time while doing only light work
that wrote to the NVMe (an `apt-get update` in a throwaway container, twice; a
journald restart, twice). It never came back by itself; the power LED stayed
green. Before the change it ran 3 h+ under far heavier load. It happened at
15 W and at 25 W, on the new supply and on the recommended charger.

- The system log is persistent now (`/var/log/journal`, created 07:11). The
  boot that ended 07:40:45 stops mid-routine with no shutdown sequence and no
  kernel error: a power cut or an unplug. Read `journalctl -b -1` after any
  new incident first.
- Not done (the session froze while setting them up): a journald drop-in
  `SyncIntervalSec=5s`, and a 2 s fsynced recorder of temps / VDD_IN / dmesg.
  Both are worth adding before load tests.
- Owner was asked to check: NVIDIA 19 V charger, plug fully in, nothing else
  on that supply, the NVMe seated with its screw.
- **Do not start this upgrade until the board survives 10 min idle and then a
  controlled write test** (e.g. `dd if=/dev/zero of=~/w.bin bs=1M count=2000
  oflag=direct; sync; rm ~/w.bin`) without freezing. Every freeze during a
  write risks the filesystem — and `orin-nav:1.1` is the only copy (§3).
- Power mode: 15 W now. Default in `/etc/nvpmodel.conf` is 25 W (owner allowed
  25 W; `sudo nvpmodel -m 1`, `-m 0` = 15 W, `-m 2` = MAXN_SUPER). The stack
  is CPU-bound (OPEN_ISSUES #1), so the mode matters — settle it once the
  power is stable, and run every A/B below in the SAME mode.

## 1. What we run and what exists

| | here | latest |
|---|---|---|
| Jetson Linux | R39.2.0 (JetPack 7.2) | 39.2.1 (Aug 2026), host apt shows it as candidate — owner's call, needs a reboot |
| Isaac ROS | **4.4.0** (2026-04-30), ROS 2 Jazzy, apt source `isaac-ros/release-4.4 noble-jetpack` in the image | **4.6.0** (2026-08-18, Jazzy); 5.0.0 (2026-09-21, **ROS 2 Lyrical**) |
| visual odometry | `phase1/nodes/vo_node.py` + the standalone **pyCuVSLAM wheel (cuVSLAM 16.0.0)** in `/opt/cuvslam`, its CUDA-12 libs first on `LD_LIBRARY_PATH` — because the packaged `isaac_ros_visual_slam` 4.4 is a **Thor build** and does not run on this Orin (ARCHITECTURE.md §0) | 4.6: "Added support for Jetson Orin", "builds cuVSLAM library from source for x86_64 and aarch64" |
| mapping | `isaac_ros_nvblox` / `nvblox_ros` 4.4.0 (`phase2/launch/nvblox.launch.py`) | 4.5: Isaac Sim semantic-label JSON support; no other nvblox change through 4.6 |
| NITROS | not used directly by our code | 4.5 sunset the GXF NITROS; 4.6 removed `NitrosCameraInfo` / `NitrosImu` wrappers (forum, 2026-08-24: use plain `sensor_msgs` types) — affects custom C++ only |

## 2. What 4.6 can and cannot give us

Can (to be measured, not assumed):
1. The packaged cuVSLAM (`isaac_ros_visual_slam`) running on this Orin Nano →
   drop the pyCuVSLAM workaround; GPU / NITROS image path → **less CPU** than
   `vo_node`'s Python copies. CPU is our #1 open issue (nav2's "no fit found in
   the time", the 250 s corner escape of 2026-10-18).
2. An officially supported Orin + JetPack 7.2 stack (future fixes target us).
3. Later options: learned stereo depth (DNN stereo decoder, 4.5: could reduce
   the passive-stereo ghosts in front of white walls — heavy for an Orin Nano;
   4.5 lists an intermittent RealSense issue), cuMotion for the future arm.

Cannot: our nvblox issues (specks / no decay by choice, < 3 cm objects, the
odom-frame drift) are config and sensor work, not version bugs.

Not certain: the 4.6 notes say "Jetson Orin", never "Orin Nano"; we found no
report of the packaged visual SLAM on an Orin Nano either way.

**Out of scope now:** Isaac ROS 5.0 (moves to ROS 2 Lyrical, removes NITROS,
renames `isaac_ros_visual_slam` → `isaac_ros_cuvslam`): it would need the
Jetson, the Pi 5 and the twin's bundled ROS migrated together.

## 3. Rules

- **Never modify, re-tag or delete `orin-nav:1.1`** (image ID `2a3d7f1d30dd…`).
  It was made by `docker commit` and cannot be rebuilt (`docker/README.md`).
  No `docker image prune`. The new image is `orin-nav:1.2`, made the same way
  (`docker commit` of a container started from 1.1). Disk: ~114 GB free; a
  1.2 layer is a few GB.
- Every test runs on the **twin first** (ROS domain 42, `TWIN=1`), then the
  real robot. The laptop: `cd ~/kino_sim && ./sim up home_real` (the twin
  announces itself; `TWIN=1 ./rover camera` finds it over DDS).
- Same power mode for every A/B. Record numbers in OPEN_ISSUES.md / here.
- Commit to this repo only what changes the robot's code; the image's apt
  source change lives in the image.

## 4. The plan

**A. Can 4.6 be installed, in a throwaway container** (`--rm`, from 1.1):
`sed -i 's#isaac-ros/release-4.4#isaac-ros/release-4.6#' /etc/apt/sources.list.d/*.list`,
`apt-get update`, `apt-cache policy ros-jazzy-isaac-ros-visual-slam
ros-jazzy-isaac-ros-nvblox ros-jazzy-nvblox-ros`. (Attempted 2026-10-18; the
board froze during `apt-get update` — see §0. Not a 4.6 result.)

**B. Does the packaged visual SLAM run and win** (throwaway container, 4.6's
`ros-jazzy-isaac-ros-visual-slam` installed, the twin's D555 live from
`TWIN=1 ./rover camera`):
- starts on the Orin Nano (no illegal instruction / Thor-only library)?
- tracks: compare with `vo_node` on the same frames — landmarks, `/vo/odom`
  vs `/sim/truth/pose`;
- CPU: `top` / `tegrastats` per process, `vo_node` vs the packaged node,
  same power mode;
- note it needs an adapter to replace `vo_node` for `fusion2`: our
  `/vo/odom`, the landmark-count health, the calibrated lever arm
  (`description/params.yaml` `vo_lever_*`), `SLAM=false` (slam_toolbox owns
  `map → odom`).

**C. nvblox 4.6** in the same container: same launch, same twin world;
`./sim mapreport home_real` vs 4.4.

**D. Build `orin-nav:1.2`** only if B or C win: container from 1.1, switch the
apt source, `apt-get install --only-upgrade` the `ros-jazzy-isaac-ros-*` /
`nvblox` packages (not a blanket upgrade), add what B needs, `docker commit`
→ `orin-nav:1.2`. Pin and record its image ID in `docker/README.md`.

**E. Full twin run:** `IMAGE=orin-nav:1.2 TWIN=1 ./rover up` — every gate;
`./sim test contract`, `encoders`, `motion`; `./sim scenario home_real` (the
2026-10-18 baseline on 1.1: 4/4 reached, pose err max 4.1-7.2 cm, door
clearance 0.7 cm, tablet driven over — kino_sim `docs/STATUS.md`).

**F. Real robot:** `IMAGE=orin-nav:1.2 ./rover up`, the usual gates, a short
`./rover record` + `./rover grade`. Rollback is `./rover stop` and plain
`./rover up` (1.1 is the default `IMAGE`).

## Sources

- Isaac ROS release notes: https://nvidia-isaac-ros.github.io/releases/index.html
- Isaac ROS 4.6 Getting Started: https://nvidia-isaac-ros.github.io/v/release-4.6/getting_started/index.html
- Isaac ROS Jetson page: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_jetson/index.html
- Forum, NITROS 4.6 missing types on Orin Nano (2026-08-22/24): https://forums.developer.nvidia.com/t/isaac-ros-nitros-4-6-missing-types/380960
- Forum, roadmap for Isaac ROS Jazzy on Orin: https://forums.developer.nvidia.com/t/roadmap-for-isaac-ros-jazzy-on-orin-devices/375982
