# Standalone cuVSLAM dev tools (no ROS)

ROS-free pyCuVSLAM experiments — useful for isolating "is cuVSLAM itself OK?" from the
ROS stack. These built the original proof that cuVSLAM runs natively on this Orin.

| File | Purpose |
|---|---|
| `Dockerfile.cuvslam-jp72` | Small dev image: Ubuntu 24.04 + CUDA 13 + librealsense **built with DDS** (ethernet D555) + the **cu12** pyCuVSLAM wheel. Build: `docker build -f Dockerfile.cuvslam-jp72 -t cuvslam-jp72:1.0 .` |
| `run_stereo_headless.py` | Live D555 stereo visual odometry, prints pose/FPS/drift (no GUI). The go-to sanity check. |
| `run_stereo_explore.py` | Same + rerun 3D viewer (`--ip <jetson-ip>` web viewer, or `--gui`). Note: rerun viewer eats ~2 GB RAM. |
| `run_vio_headless.py` | Stereo+IMU (VIO) experiment — **diverges** (D555 DDS driver gives identity IMU extrinsics); kept for when Kalibr calibration happens. |

Key facts baked into these scripts: D555 native IR profile **896×504@30** (y8), DDS context
`{"dds":{"enabled":true,"domain":0}}`, cu13 wheel SIGILLs on Orin — only **cu12** works.

Run pattern:
```bash
docker run -it --rm --runtime=nvidia --network=host --privileged \
  -e NVIDIA_DISABLE_REQUIRE=1 -v $(pwd)/..:/w cuvslam-jp72:1.0 \
  python3 /w/standalone/run_stereo_headless.py
```
(The `cuvslam-jp72:1.0` image was deleted in the 2026-07-19 cleanup — rebuild from the
Dockerfile here if needed; ~25 min.)
