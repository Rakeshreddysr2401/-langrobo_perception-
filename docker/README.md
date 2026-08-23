# docker/ — provenance of the image, NOT a build recipe

**Nothing here has been built or tested from this repo.** These files are a
*record* of where `orin-nav:1.1` came from, copied in on 2026-08-11 as insurance
against `../langrobo_perception/` disappearing. `rover` does not read them.

## The image `rover` actually runs

| | |
|---|---|
| Tag | `orin-nav:1.1` |
| Image ID | `sha256:2a3d7f1d30dde60397899073ff8d291dbb75b57a0a042621aee5e84640e3741d` |
| Built | 2026-07-19 16:35 (+05:30) |
| Size | 57.8 GB (21.6 GB unique) |

Pin the ID, not the tag: `orin-nav:0.0.2` also exists, is also 57.8 GB, and is a
**different image** (`e1c32d08f6c6`). A tag is a movable label; the ID is not.

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
