# Headless D555 -> cuVSLAM stereo visual-odometry smoke test (no rerun GUI).
#
# Same pipeline as run_stereo.py, but prints pose/FPS instead of spawning the
# rerun viewer, so it works over SSH / in an agent session. Use this to confirm
# cuVSLAM initializes on the Orin (no SIGILL / tracker-init failure) and tracks
# the ethernet D555 stereo pair. Ctrl-C to stop.
#
#   python3 run_stereo_headless.py [--width W] [--height H] [--fps F] [--frames N]
import argparse
import time
from typing import List, Optional

import numpy as np
import pyrealsense2 as rs
import cuvslam as vslam
from camera_utils import get_rs_stereo_rig


def make_context() -> "rs.context":
    """Context with DDS enabled (domain 0) so the ETHERNET D555 is discovered.
    Falls back to a default context if this librealsense lacks DDS settings."""
    for settings in ('{"dds":{"enabled":true,"domain":0}}', ''):
        try:
            return rs.context(settings) if settings else rs.context()
        except Exception as e:
            print(f"[info] context('{settings}') failed: {e}")
    return rs.context()


def list_ir_profiles(ctx: "rs.context") -> None:
    """Print the infrared stereo profiles the connected RealSense actually offers."""
    devs = ctx.query_devices()
    print(f"[info] RealSense devices found: {len(devs)}")
    for d in devs:
        name = d.get_info(rs.camera_info.name)
        serial = d.get_info(rs.camera_info.serial_number)
        print(f"[info] device: {name}  S/N {serial}")
        for s in d.query_sensors():
            for p in s.get_stream_profiles():
                if p.stream_type() == rs.stream.infrared:
                    v = p.as_video_stream_profile()
                    print(f"        IR{p.stream_index()} {v.width()}x{v.height()} "
                          f"{p.fps()}fps {p.format()}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--width", type=int, default=896)   # D555 native stereo IR
    ap.add_argument("--height", type=int, default=504)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--frames", type=int, default=0, help="stop after N tracked frames (0 = run forever)")
    ap.add_argument("--warmup", type=int, default=60)
    args = ap.parse_args()

    ctx = make_context()
    list_ir_profiles(ctx)

    config = rs.config()
    pipeline = rs.pipeline(ctx)
    config.enable_stream(rs.stream.infrared, 1, args.width, args.height, rs.format.y8, args.fps)
    config.enable_stream(rs.stream.infrared, 2, args.width, args.height, rs.format.y8, args.fps)

    # Prime: one start/stop to read intrinsics+extrinsics.
    try:
        pipeline.start(config)
    except Exception as e:
        print(f"[FAIL] could not start {args.width}x{args.height}@{args.fps} IR stereo: {e}")
        print("[hint] pick a width/height/fps from the IR profiles listed above.")
        return
    frames = pipeline.wait_for_frames()
    pipeline.stop()

    camera_params = {'left': {}, 'right': {}}
    left_profile = frames[0].profile.as_video_stream_profile()
    right_profile = frames[1].profile.as_video_stream_profile()
    camera_params['left']['intrinsics'] = left_profile.intrinsics
    camera_params['right']['intrinsics'] = right_profile.intrinsics
    camera_params['right']['extrinsics'] = right_profile.get_extrinsics_to(left_profile)

    cfg = vslam.Tracker.OdometryConfig(
        async_sba=False,
        enable_final_landmarks_export=True,
        enable_observations_export=True,
        rectified_stereo_camera=True,
    )
    rig = get_rs_stereo_rig(camera_params)
    tracker = vslam.Tracker(rig, cfg)   # <-- cuVSLAM init happens here (Orin make-or-break)
    print("[ok] cuVSLAM tracker constructed on this device")

    profile = pipeline.start(config)
    device = profile.get_device()
    depth_sensor = device.query_sensors()[0]
    if depth_sensor.supports(rs.option.emitter_enabled):
        depth_sensor.set_option(rs.option.emitter_enabled, 0)

    frame_id = 0
    tracked = 0
    valid = 0
    prev_ts: Optional[int] = None
    t_last = time.time()
    trajectory: List[np.ndarray] = []
    JITTER_NS = 35 * 1e6

    try:
        while True:
            fr = pipeline.wait_for_frames()
            lf = fr.get_infrared_frame(1)
            rf = fr.get_infrared_frame(2)
            if not lf or not rf:
                print("[warn] missing frame")
                continue
            frame_id += 1
            ts = int(lf.timestamp * 1e6)
            if prev_ts is not None and (ts - prev_ts) > JITTER_NS:
                print(f"[warn] stream gap {(ts-prev_ts)/1e6:.1f} ms")
            prev_ts = ts

            images = (np.asanyarray(lf.get_data()), np.asanyarray(rf.get_data()))

            if frame_id > args.warmup:
                est, _ = tracker.track(ts, images)
                tracked += 1
                if est.world_from_rig is None:
                    print("[warn] pose not valid")
                else:
                    valid += 1
                    p = est.world_from_rig.pose
                    trajectory.append(p.translation)
                    if valid % 30 == 0:
                        now = time.time()
                        fps = 30.0 / (now - t_last) if now > t_last else 0.0
                        t_last = now
                        tr = p.translation
                        print(f"[track] valid={valid} fps~{fps:4.1f} "
                              f"pos=({tr[0]:+.3f}, {tr[1]:+.3f}, {tr[2]:+.3f}) m")
                if args.frames and tracked >= args.frames:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.stop()
        print(f"[done] frames={frame_id} tracked={tracked} valid_poses={valid} "
              f"path_len={len(trajectory)}")


if __name__ == "__main__":
    main()
