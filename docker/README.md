# docker/ — provenance of the image, NOT a build recipe

**Nothing here has been built or tested from this repo.** These files are a
*record* of where `orin-nav:1.1` came from, copied in on 2026-08-11 as insurance
against `../langrobo_perception/` disappearing. `rover` does not read them.

## The image `rover` actually runs

Since 2026-10-18 the robot runs `orin-nav:1.2`; the twin stays on 1.1 until it
is moved on purpose. `IMAGE=orin-nav:1.1 ./rover up` is the rollback.

| | |
|---|---|
| Tag | `orin-nav:1.2` |
| Image ID | `sha256:46c9e69f6975bc6107962028ae4078b5936ddb62723d4a3eb671eb532d4b828e` |
| Built | 2026-10-18 (+05:30), by `docker commit`, from 1.1 |
| Size | 73.4 GB (layers on top of 1.1's) |

What changed from 1.1 (ISAAC_ROS_4_6_UPGRADE.md has the why):

- Isaac ROS 4.4 -> **4.6** (`ros-jazzy-isaac-ros-*`, `nvblox-*` only; the 21
  GXF packages 4.6 retired stay at 4.4, unused). Its `isaac_ros_visual_slam`
  runs on this Orin -- 4.4's was a Thor build -- and is the rover's VO now
  (`phase1/launch/vo_isaac.launch.py` + `phase1/nodes/vo_isaac.py`).
- CUDA **13.2** next to 13.0 (`/usr/local/cuda` -> 13.2), from the Jetson
  repo, which is now `jetson/common r39.2` (was r38.4 -- JetPack 7.0, Thor's).
- `isaac-ros-cli` 2.3 -> 2.5 with `ISAAC_ROS_PLATFORM=arm64-jetpack`, so its
  apt pins are the 4.6 Jetson ones (CUDA 13.2, TensorRT 10.16). The 4.4 pins
  it replaced are kept at `/root/apt-prefs-4.4/`. TensorRT itself is still
  10.13: nothing needed the upgrade.
- **pyCuVSLAM removed**: the `cuvslam` 16.0.0 wheel, its seven
  `nvidia-*-cu12` pip libs and `/opt/cuvslam`. `CUVSLAM_CU12_LIBS` is empty.

The intermediate 1.2 that still had the wheel was `16f2d70ed8e8`; the image
above is a layer on it.

| | |
|---|---|
| Tag | `orin-nav:1.1` (rollback; the twin) |
| Image ID | `sha256:2a3d7f1d30dde60397899073ff8d291dbb75b57a0a042621aee5e84640e3741d` |
| Built | 2026-07-19 16:35 (+05:30) |
| Size | 57.8 GB (21.6 GB unique) |

Pin the ID, not the tag: a tag is a movable label; the ID is not. (`orin-nav:0.0.2`,
`e1c32d08f6c6`, was 1.1 plus a 1.92 MB layer of runtime leftovers -- ld.so cache,
`.realsense-config.json`, .pyc files; its work is orin-nav-stack commit `5adf4e0`.
Nothing used it; deleted 2026-10-18.)

## Why the Dockerfile here does not save you

`Dockerfile` is verbatim from `../langrobo_perception/orin-nav-stack/Dockerfile`.
Read it: it is fifteen lines of `COPY` on top of

```
FROM isaac_ros:cuvslam-unified
```

and **that** is where the 57.8 GB lives. The chain, from `docker history` and the
image labels:

```
orin-nav:1.1
  └─ isaac_ros:cuvslam-unified   (57.8 GB)   ← no Dockerfile exists, anywhere
       └─ isaac_ros:langrobo-prod (54.4 GB)  ← no Dockerfile exists, anywhere
```

`orin-nav:1.1` carries `com.docker.compose.*` labels naming project `robot` and
config `/home/rakhi24/robot/docker-compose.yml`. Those labels are the fingerprint
of an image produced by **`docker commit` on a running container**, not by a
build. So the base was never described by a recipe and cannot be reproduced from
one — not with this file, not with anything in the old repo.

⚠️ **Therefore: the only real backup of this stack is an image export, not a
Dockerfile.** See `TODO.md §13`.

## One more caveat

The copied `Dockerfile` has mtime 2026-07-19 **16:48**; the image was built at
**16:35**. It was edited 13 minutes *after* the build, so it is not guaranteed to
be the exact source of the running image.

## The other Dockerfile in the old repo

`orin-nav-stack/standalone/Dockerfile.cuvslam-jp72` **is** a real, complete,
from-scratch recipe (`FROM nvidia/cuda:13.0.0-devel-ubuntu24.04`, librealsense
2.58.2 with `BUILD_WITH_DDS=ON`, the cu12 cuVSLAM wheel). It is not deliberately
copied here because it builds a *different, standalone* image — cuVSLAM only, no
nvblox, no nav2. If the base image is ever lost, that file is the closest thing
to a starting point for rebuilding, and it is worth reading before you need it.
