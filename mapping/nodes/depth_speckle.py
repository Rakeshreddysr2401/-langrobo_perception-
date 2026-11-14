"""depth_speckle.py — drop the stereo camera's speckle before it reaches the map (2026-11-14).

Measured on the real rover, 2026-11-14 (kino_sim tools/depth_vs_lidar.py, speckle_eval.py; 634 frames
judged against the LiDAR of the same moment): with the emitter off (it must be, for visual odometry)
a plain surface -- the white wall, the dark door, anything nearer than the camera's minimum range --
has no texture to match, and the D555 answers with small scattered blobs at RANDOM ranges (0.3-3 m).
At the door, pinned, a frame was 3-5 % blobs and nothing else. nvblox painted every blob: after two
short drives 2,311 of its 4,727 cells near the rover stood BEHIND a surface the LiDAR hits, and the
planner, the route driver and nav2 all refused a 56 cm doorway that was open.

A real surface is a large patch whose disparity changes smoothly; a real thing on the floor is joined
to the floor's patch where it stands. So: label patches of smoothly changing disparity and zero the
small ones (the classic stereo speckle filter, cv2.filterSpeckles). On those 634 frames, patches
under 2500 px (of 896x504) removed: camera points behind the LiDAR's surface 128,057 -> 2,986
(2.3 % left); floor kept 94 %; points on the LiDAR's surface kept 85 % (the LiDAR has those anyway).

Full resolution on purpose: at half resolution (4x cheaper) the floor's patch broke up and only 81 %
of it survived. Cost on the Jetson: ~9 ms a frame.
"""
import cv2
import numpy as np

BASELINE_M = 0.095      # D555 stereo baseline (datasheet); only scales the disparity step below
MAX_STEP_PX = 1.0       # neighbours belong to one patch if their disparity differs by less (px)
MIN_PATCH_PX = 2500     # patches smaller than this are speckle; 0 = filter off


def clean(depth_mm, fx, min_patch_px=MIN_PATCH_PX):
    """depth_mm: HxW uint16 (0 = no depth), CHANGED IN PLACE: pixels of small patches set to 0.
    Returns how many valid pixels were removed."""
    if min_patch_px <= 0:
        return 0
    ok = depth_mm > 0
    disp = np.zeros(depth_mm.shape, np.int16)                   # disparity in 1/16 px
    disp[ok] = np.clip(fx * BASELINE_M * 16000.0 / depth_mm[ok].astype(np.float32), 1, 32000).astype(np.int16)
    cv2.filterSpeckles(disp, 0, int(min_patch_px), int(MAX_STEP_PX * 16))
    bad = ok & (disp == 0)
    removed = int(np.count_nonzero(depth_mm[bad]))
    depth_mm[bad] = 0
    return removed


def clean_cdr(data, fx, min_patch_px=MIN_PATCH_PX):
    """A serialized sensor_msgs/Image (16UC1, little-endian CDR) -> (the same message with its depth
    cleaned, pixels removed). Anything unexpected (other size, other layout) passes untouched, (data, -1):
    the map must not go blind because a header changed."""
    if min_patch_px <= 0:
        return data, 0
    try:
        n = int.from_bytes(data[12:16], 'little')               # frame_id length (with its 0)
        off = 16 + n
        off += (-(off - 4)) % 4                                 # CDR aligns to 4 from the end of the 4-byte header
        h = int.from_bytes(data[off:off + 4], 'little'); w = int.from_bytes(data[off + 4:off + 8], 'little')
        start = len(data) - h * w * 2
        if h <= 0 or w <= 0 or start < off + 8 or int.from_bytes(data[start - 4:start], 'little') != h * w * 2:
            return data, -1
    except Exception:
        return data, -1
    buf = bytearray(data)
    img = np.frombuffer(buf, np.uint16, h * w, start).reshape(h, w)
    removed = clean(img, fx, min_patch_px)
    return bytes(buf), removed
