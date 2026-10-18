# Isaac ROS 4.4 → 4.6 on this Jetson — the plan, for whoever picks it up

Written 2026-10-18 (laptop session, for the agent working here). Owner's goal:
get onto the release that officially supports this board, and see whether the
packaged cuVSLAM can replace our pyCuVSLAM workaround — **without risking the
working robot**. Nothing below has been done yet except where it says so.

> **2026-10-18, later:** the laptop's twin is being rebuilt on Isaac Sim 6.2 + Isaac Lab 3.0
> (the Isaac Sim 5.1 stack was removed; the old twin is git tag `isaacsim5-final` in
> kino_sim). Until the new twin passes its checks, only step **A** below can run; **B, C and
> E wait** for it. The twin keeps the same cables and topics, so nothing here changes.

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
`./rover record` + `./rover grade`. Rollback is `./rover stop` and
`IMAGE=orin-nav:1.1 ./rover up` (1.2 is the default `IMAGE` since 2026-10-18).

## Sources

- Isaac ROS release notes: https://nvidia-isaac-ros.github.io/releases/index.html
- Isaac ROS 4.6 Getting Started: https://nvidia-isaac-ros.github.io/v/release-4.6/getting_started/index.html
- Isaac ROS Jetson page: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_jetson/index.html
- Forum, NITROS 4.6 missing types on Orin Nano (2026-08-22/24): https://forums.developer.nvidia.com/t/isaac-ros-nitros-4-6-missing-types/380960
- Forum, roadmap for Isaac ROS Jazzy on Orin: https://forums.developer.nvidia.com/t/roadmap-for-isaac-ros-jazzy-on-orin-devices/375982

## 5. Done 2026-10-18 (Jetson session) — read this first

Owner's calls: skip the §0 stress test; standardise on the packaged Isaac
cuVSLAM and drop pyCuVSLAM; **no twin tests** (kino_sim is mid-development) —
the real-robot test comes next, after a power cycle to connect the D555/C1.

- **Binary check before anything:** 4.4's `libcuvslam.so` has ~5280 SVE
  instructions (the SIGILL); 4.6's has 0 and carries sm_87 + sm_110 GPU code.
  The "Orin Nano?" question of §2 is answered: it runs here.
- **`orin-nav:1.2` = `46c9e69f6975`** — what changed is in `docker/README.md`.
  1.1 untouched. A mains power cut at 08:35 interrupted the install (not a
  board freeze); it was resumed from the apt cache.
- **4.6 needed CUDA 13.2**: the 4.4 `isaac-ros-cli` pins block 13.1+, so the
  CLI went to 2.5 with `ISAAC_ROS_PLATFORM=arm64-jetpack` and the Jetson repo
  to r39.2, exactly as 4.6's own `Dockerfile.isaac_ros` does.
- **The VO switch** (`phase1/launch/vo_isaac.launch.py`, `phase1/nodes/vo_isaac.py`,
  `./rover pose` with `VO=isaac`, the default on 1.2). vo_isaac.py keeps
  vo_node's contract: `/vo/status` with the same keys, the z-divergence guard
  (-> `/visual_slam/reset`), the floor-projected, gated map -> odom (slam:=true
  only). The Isaac node publishes `/vo/odom` itself; it tracks in
  `vslam_{left,right}_optical`, published under camera0_link with vo_node's
  axis swap and camera_info's |baseline| — not the driver's infra TF.
- **Synthetic-stereo test** (textured wall at 2 m, rig sliding 0.10 m/s; no
  camera, no twin): 30 Hz, 0.100 ± 0.002 m/s tracked, < 1 cm cross-axis,
  ~210 landmarks, healthy. CPU, % of one core, same feed:

  | | SLAM on | SLAM off |
  |---|---|---|
  | vo_node + pyCuVSLAM 16.0 | 29 | 26 |
  | Isaac 4.6 (cuVSLAM 15.0) + vo_isaac.py | 28 + 5 | 27 + 3 |

  The SLAM backend runs in both modes: Isaac publishes the landmark cloud only
  from it, and fusion2 drops VO under 20 landmarks. vo_isaac.py reads its
  topics raw and spins the events executor — the first version cost 22 %.
  **No CPU win on this feed**; the real D555 path (cv_bridge copies in
  vo_node) may differ. Measure it.
- **Not yet known — the real-robot test must answer:** tracking vs LiDAR truth
  (`./rover record` + `./rover grade`), real CPU, the D555 with emitter off,
  `ground:=true` (enable_ground_constraint_in_odometry) as an option.
- Host: the 2026-07-16 `apt upgrade` had been cut off (230 packages half
  configured, openssh/systemd/docker among them) — repaired. The host's
  `isaac-ros-cli` 2.3's pins forced Ubuntu's OpenCV 4.6 over JetPack's 4.8
  and held CUDA at 13.0. The owner keeps the CLI installed-but-dormant, so it
  is now **2.5 from release-4.6** (`ISAAC_ROS_PLATFORM=arm64-jetpack`) with
  its OpenCV pin disabled and a pin keeping Ubuntu's python3-rich
  (orin-nav-stack README §12 has the detail).
- **JetPack 7.2.1 installed** (510 packages, 14:20-14:28); bootloader capsule
  applied on the next boot (39.2.1, slot B). Host: 0 updates pending.
- **JetPack 7.2.1 regression, fixed in `./rover`:** `libnvcuvid.so.1` moved
  to `/opt/nvidia/l4t-gpu-libs/openrm/` and is now handed to containers only
  on the openrm driver (this Orin is nvgpu). The images' VPI / NITROS -- so
  visual_slam and nvblox -- would not load. `start_container` bind-mounts it
  back to where 39.2.0 had it.
- **Real robot on 1.2 (15:00, standing still):** camera 30 Hz, Isaac VO 30 Hz
  healthy (111-507 landmarks, ~3 mm in 20 s), lidar 10 Hz, fused 20 Hz,
  health.py all ok, fusion accepted 115 VO fixes / rejected 0. CPU: tracker
  11 % + vo_isaac 3 % of a core (synthetic feed: 27 + 3).
- **nvblox 4.6 regression, fixed in `./rover`:** its `libnvblox_ros_lib.so`
  (280 MB) carries GPU code for **sm_75 only** -- 4.4's (1.1) had sm_87. The
  driver JIT-compiles it from PTX on first use: ~4 min at 100 % CPU, during
  which the D555's IR/depth collapsed (twice; `./rover camera` recovered it).
  The JIT cache lived in the container, so EVERY bring-up paid it.
  `start_container` now mounts `~/.cache/rover-cuda/$NAME` on
  `/root/.nv/ComputeCache` with `CUDA_CACHE_MAXSIZE=4 GB`: compiled once
  (47 MB), a fresh container then maps straight away. Report it upstream.
- **Full stack on 1.2, standing still (16:08):** IR 28, VO 25 (healthy),
  gyro 200, scan 10, wheels 20, fused 20, depth->map 15.6, grid 4.8,
  YOLO (cuda) 5.1 Hz; health.py all ok, fusion VO fixes 1726 accepted /
  0 rejected. **Still to run:** a driven `record` + `grade` (owner's OK).
- Lesson: `ros2 topic hz` on 896x504 images UNDER-reports on a loaded Orin
  (Python cannot keep up) -- judge the camera by `/vo/status` rate_hz and
  `/depth_gate/passed`, small messages.
- `orin-nav:0.0.2` deleted (1.1 + a 1.92 MB layer of runtime leftovers;
  nothing used it). 1.1 stays: rollback and the twin.
- **The "D555 goes half-alive" episodes were not the camera (16:16-16:50).**
  IR (then depth) stopped a minute or two into every bring-up while
  `camera_info` kept 30 Hz. A probe with `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`
  got IR and depth at 30 Hz from the same driver at the same moment; the
  default (shared-memory) probe got nothing: a full 512 KB SHM segment, the
  2026-10-10 "UDP-only worked" failure again. `ENVSET` now sets UDPv4 for
  everything in the container. Same day: the colour profile `424x240x15`
  (a D4xx one, rejected since 2026-09-06) silently ran colour at 896x504x30,
  ~40 MB/s on a link already carrying 54; it is `448x252x15` now (link 82 ->
  58 MB/s). After both, the full stack ran IR 30.2, depth 30.2, VO 30.1 Hz.
- `./rover view` took the login screen's Xwayland (gdm, `:1024`) for ours and
  died with "Authorization required"; it now only looks at the user's own
  Xwayland and sessions, and says "nobody is logged in" when that is true.

## 6. Step E done 2026-10-18 (laptop session) — the twin on 1.2

Twin = kino_sim `main` on Isaac Sim 6.2 (the 5.1 twin is retired). Jetson in 15 W.

- `IMAGE=orin-nav:1.2 TWIN=1 ./rover up`: **every gate OK** (camera, lidar, pose
  with VO=isaac, fused, slam, map, nav, vlm, wheels). The first start in a fresh
  `rover_twin` cache failed the map gate: nvblox JIT-compiles ~27 s per layer
  (~3 min) into `~/.cache/rover-cuda/rover_twin`; a re-run of `./rover up` passed
  and later starts reuse the cache. (The cache dirs are root-owned: `du` as
  rakhi24 reads 48K -- that is permissions, not an empty cache.)
- `./sim test contract | encoders | motion`: PASS.
- `./sim scenario home_real`, 1.2 vs 1.1 (same day, same twin):

  | goal | 1.1 (VO=pycuvslam) | 1.2 (VO=isaac) |
  |---|---|---|
  | 1 (3.05, 0.1) | 53 s, over the tablet, pose err 7.5 cm | 67 s, over the tablet, 5.5 cm |
  | 2 (5.2, 0.9), the 80 cm door | 106 s, 1.2 cm from the wall, 3.5 cm | 93 s, **0.2 cm** from the frame, 5.8 cm |
  | 3 (5.6, -0.2) | 28 s, 35 cm clear, 10.7 cm | 20 s, 35 cm clear, 2.7 cm |
  | 4 (0.3, -0.45, 180) | 70 s, over the tablet, 9.1 cm | 58 s, over the tablet, 10.0 cm |

  4/4 reached on both; the tablet (OPEN_ISSUES #3) as on every twin since 10-11.
  Open: the door pass at 0.2-1.2 cm (5.1 twin: 2.8 cm) and 93-106 s (5.1: 27 s).
- `./rover` defaults to 1.2 for the twin AND the real robot (`IMAGE=${IMAGE:-orin-nav:1.2}`):
  the owner's decision, 2026-10-18 ("go ahead with 1.2"). 1.1 is the rollback only
  (`IMAGE=orin-nav:1.1 ./rover up`). The owner
  deletes 1.1 after one more real-robot run on 1.2 (step F).
- Twin camera rate: stereo 20-25 Hz, not 30 -- the twin's own limit (Python
  stand-in driver + laptop->router->Jetson WiFi ~6 MB/s), not the robot's.
