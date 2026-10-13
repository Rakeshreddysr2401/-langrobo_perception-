"""person_look.py — what a person LOOKS like, as a few numbers: their clothes' colours.

WHY (2026-10-13, owner: "even when he goes out of the box it needs to search
here and there and find and follow him"). The tracker knows people only by
where they stand, and a person who leaves the view comes back with a new id
somewhere else. Following on means telling THAT person from anyone else in
the room. The measured choices for a real re-ID model on this Orin are poor
(no TensorRT in the image, and the image is never modified), so this is the
classic cheap cue: the colour of the top and of the trousers.

look_of(img, box) -> 22 numbers: two body bands (top 15-50 % of the box, the
torso; 50-85 %, the legs), central 60 % of the width (the background at the
sides is not them), each band an 11-bin histogram summing to 1:
  8 hue bins over the coloured pixels (saturation and value high enough to
    have a hue), and 3 brightness bins (dark / mid / bright) for the rest --
    black, grey and white clothes have no hue.
Compared in phase3/nodes/follow.py (look_sim: the histogram intersection,
averaged over the bands; 1 = the same colours) -- it is the only consumer, and
phase3 cannot import from here inside the container. Keep BINS / BANDS in step.

Close up (under ~1.2 m) the box is only the upper body, so the bands shift:
the follower only ever compares, it never trusts one number alone.
"""
import numpy as np

try:
    import cv2
except ImportError:            # a test box without OpenCV: no looks
    cv2 = None

BANDS = ((0.15, 0.50), (0.50, 0.85))
HUE_BINS = 8
SAT_MIN, VAL_MIN = 60, 40      # below: no meaningful hue
DARK, BRIGHT = 70, 170         # value thresholds of the grey bins
BINS = HUE_BINS + 3
MIN_H, MIN_W = 40, 15          # px: a smaller box has too few pixels to say anything


def look_of(img, box):
    """img BGR uint8, box (u1, v1, u2, v2). A list of 2 * BINS floats, or None."""
    if cv2 is None or img is None:
        return None
    u1, v1, u2, v2 = (int(round(c)) for c in box)
    h, w = v2 - v1, u2 - u1
    if h < MIN_H or w < MIN_W:
        return None
    x1, x2 = max(0, u1 + int(0.2 * w)), min(img.shape[1], u2 - int(0.2 * w))
    out = []
    for a, b in BANDS:
        y1, y2 = max(0, v1 + int(a * h)), min(img.shape[0], v1 + int(b * h))
        if y2 - y1 < 4 or x2 - x1 < 4:
            return None
        hsv = cv2.cvtColor(img[y1:y2, x1:x2], cv2.COLOR_BGR2HSV).reshape(-1, 3)
        hue, sat, val = hsv[:, 0], hsv[:, 1], hsv[:, 2]
        col = (sat >= SAT_MIN) & (val >= VAL_MIN)
        hist = np.zeros(BINS)
        hist[:HUE_BINS] = np.bincount((hue[col].astype(int) * HUE_BINS) // 180,
                                      minlength=HUE_BINS)[:HUE_BINS]
        grey = val[~col]
        hist[HUE_BINS] = np.count_nonzero(grey < DARK)
        hist[HUE_BINS + 1] = np.count_nonzero((grey >= DARK) & (grey < BRIGHT))
        hist[HUE_BINS + 2] = np.count_nonzero(grey >= BRIGHT)
        s = hist.sum()
        if s <= 0:
            return None
        out.extend(np.round(hist / s, 3).tolist())
    return out
