# Live D555 -> cuVSLAM explorer with a rerun WEB viewer served over the LAN.
# Open http://<jetson-ip>:<web-port> in a browser on any device on the network
# and watch the camera image, 3D trajectory, landmarks and observations update
# live as you move the D555. Same tracking as run_stereo.py; only the rerun sink
# is changed from a local spawned window to a network web viewer.
#
#   python3 run_stereo_explore.py --ip 192.168.1.15
import argparse
import time
from typing import List

import numpy as np
import pyrealsense2 as rs
import cuvslam as vslam
import rerun as rr

from camera_utils import get_rs_stereo_rig
from run_stereo_headless import make_context, list_ir_profiles

# rerun 0.33 removed set_time_sequence (used by the SDK's visualizer.py) in favor
# of the unified set_time(timeline, sequence=...). Shim it back.
if not hasattr(rr, "set_time_sequence"):
    rr.set_time_sequence = lambda timeline, seq: rr.set_time(timeline, sequence=seq)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="192.168.1.15",
                    help="Jetson LAN IP the browser will reach (wlan 192.168.1.15)")
    ap.add_argument("--width", type=int, default=896)    # D555 native stereo IR
    ap.add_argument("--height", type=int, default=504)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--grpc-port", type=int, default=9876)
    ap.add_argument("--web-port", type=int, default=9090)
    ap.add_argument("--warmup", type=int, default=60)
    ap.add_argument("--gui", action="store_true",
                    help="open a native rerun window on the local display ($DISPLAY) "
                         "instead of serving a web viewer")
    args = ap.parse_args()

    if args.gui:
        # Native window on the Jetson's own monitor: let RerunVisualizer.__init__
        # do rr.init(spawn=True) so the rerun viewer opens on $DISPLAY.
        print("[gui] opening native rerun window on local display", flush=True)
    else:
        # --- rerun web viewer over the LAN ---
        rr.init("cuVSLAM D555")
        rr.serve_grpc(grpc_port=args.grpc_port)
        rr.serve_web_viewer(web_port=args.web_port, open_browser=False,
                            connect_to=f"rerun+http://{args.ip}:{args.grpc_port}/proxy")
        print(f"\n============  OPEN IN BROWSER:  http://{args.ip}:{args.web_port}  ============\n",
              flush=True)
        time.sleep(1)
        # RerunVisualizer.__init__ calls rr.init(spawn=True); we already serve, so
        # neutralize re-init to keep the web sink.
        rr.init = lambda *a, **k: None

    from visualizer import RerunVisualizer

    # --- D555 stereo pipeline over DDS/ethernet ---
    ctx = make_context()
    list_ir_profiles(ctx)
    config = rs.config()
    pipeline = rs.pipeline(ctx)
    config.enable_stream(rs.stream.infrared, 1, args.width, args.height, rs.format.y8, args.fps)
    config.enable_stream(rs.stream.infrared, 2, args.width, args.height, rs.format.y8, args.fps)

    pipeline.start(config)
    frames = pipeline.wait_for_frames()
    pipeline.stop()

    camera_params = {'left': {}, 'right': {}}
    lp = frames[0].profile.as_video_stream_profile()
    rp = frames[1].profile.as_video_stream_profile()
    camera_params['left']['intrinsics'] = lp.intrinsics
    camera_params['right']['intrinsics'] = rp.intrinsics
    camera_params['right']['extrinsics'] = rp.get_extrinsics_to(lp)

    cfg = vslam.Tracker.OdometryConfig(
        async_sba=False,
        enable_final_landmarks_export=True,
        enable_observations_export=True,
        rectified_stereo_camera=True,
    )
    rig = get_rs_stereo_rig(camera_params)
    tracker = vslam.Tracker(rig, cfg)
    viz = RerunVisualizer()

    profile = pipeline.start(config)
    dev = profile.get_device()
    ds = dev.query_sensors()[0]
    if ds.supports(rs.option.emitter_enabled):
        ds.set_option(rs.option.emitter_enabled, 0)

    frame_id = 0
    valid = 0
    trajectory: List[np.ndarray] = []
    t_last = time.time()
    try:
        while True:
            fr = pipeline.wait_for_frames()
            lf = fr.get_infrared_frame(1)
            rf = fr.get_infrared_frame(2)
            if not lf or not rf:
                continue
            frame_id += 1
            ts = int(lf.timestamp * 1e6)
            images = (np.asanyarray(lf.get_data()), np.asanyarray(rf.get_data()))
            if frame_id > args.warmup:
                est, _ = tracker.track(ts, images)
                if est.world_from_rig is None:
                    print("[warn] pose not valid", flush=True)
                    continue
                pose = est.world_from_rig.pose
                trajectory.append(pose.translation)
                viz.visualize_frame(
                    frame_id=frame_id,
                    images=[images[0]],
                    pose=pose,
                    observations_main_cam=[tracker.get_last_observations(0)],
                    trajectory=trajectory,
                    timestamp=ts,
                )
                valid += 1
                if valid % 30 == 0:
                    now = time.time()
                    fps = 30.0 / (now - t_last) if now > t_last else 0.0
                    t_last = now
                    tr = pose.translation
                    print(f"[track] valid={valid} fps~{fps:4.1f} "
                          f"pos=({tr[0]:+.3f},{tr[1]:+.3f},{tr[2]:+.3f}) m", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.stop()


if __name__ == "__main__":
    main()
