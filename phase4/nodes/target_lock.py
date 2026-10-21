"""target_lock.py — keep seeing ONE locked thing while the rover moves (no ROS).

    t = Target(lock_bgr, box, point_odom, cam_at_lock, K, now)        # find it once
    s = t.update(bgr, depth_m, cam_now, K, now)                        # every frame
    s["state"]  tracking | coasting | occluded | out_of_view | too_close | lost
    s["object"] the odom point, refined by every good sighting

WHY (CLOSED_LOOP_PLAN.md, 2026-10-21). "Go near X" was one shot: one photo,
one VLM pixel, one 3D point, then drive blind. A first estimate 20 cm off
(VLM y error, a depth hole, the background behind a thin object) or an object
that moved was never corrected. Here the thing is found ONCE -- by the VLM, a
detector box or a tap -- and then followed by what it looks like and where it
is, not by its name, so "that dot" works the same as "the bottle".

HOW, each frame (no detector, no VLM: ~1 ms of OpenCV on a small window)
  1. PREDICT  the stored odom point through the camera pose of THIS frame
              (fusion2's pose is good to ~1 cm, so the prediction is tight).
  2. MATCH    the lock-time look (a grey template, rescaled for the new range)
              in a window around the prediction, sized by how sure the point
              is (`sigma`, metres, shown in pixels at this range). Scores are
              weighed by distance from the prediction, and a second peak
              nearly as good elsewhere = ambiguous = no update (a look-alike
              beside it must not steal the lock).
  3. MEASURE  the 3D point at the match: aligned depth in the box's centre,
              preferring depths near the predicted range (a thin object's box
              is mostly background). No usable depth -> a thing locked on the
              floor uses the floor plane (z = 0: the 1 cm tablet, a dot);
              anything else keeps its range and takes the new bearing.
  4. GATE     a point further than max(GATE_MIN_M, 3 sigma) from the stored
              one is an outlier and ignored.
  5. FUSE     the stored point moves ALPHA of the way to the measurement and
              sigma shrinks. Unseen, sigma grows -- the search window AND the
              gate widen together, so a thing that was moved (30 cm) is found
              again where it now is within ~2 s; "from_lock_m" says how far the
              point has come from the first guess.

A lock is never dropped here. "lost" (in view, unseen LOST_S) is the caller's
cue for one VLM check ("is it still there?"); a sighting returns it to
"tracking". "too_close" (nearer than NEAR_M, or bigger than the frame) and
"out_of_view" keep the stored point: the last metre is driven on it, because a
low thing leaves the D555's view before the rover arrives.

Frames: odom (as the rest of the stack); the camera pose is the COLOUR
OPTICAL frame in odom (x right, y down, z forward), as a rotation R and
translation t: p_odom = R @ p_cam + t.
"""

import math

import cv2
import numpy as np

# Matching
ACCEPT = 0.55          # min normalised correlation to call it a sighting
STRONG = 0.80          # a sighting this good may also refresh the look
AMBIG_MARGIN = 0.06    # a second peak within this of the best, elsewhere = ambiguous
DIST_PENALTY = 0.15    # score lost at the edge of the search window (prefers the prediction)
TEMPLATE_MIN_PX = 8    # smaller than this on screen: too far to match
TEMPLATE_MAX_PX = 96   # the stored look is shrunk to this (cost, and blur tolerance)
MIN_VISIBLE = 0.95     # matching needs the whole look on screen; less = out of view
MIN_BOX_PX = 14        # a smaller lock box is widened to this, centred: context to match on
SEARCH_MIN_PX = 24
SEARCH_MAX_PX = 180
ADAPT = 0.10           # look refresh weight on a STRONG sighting (viewpoint drift)

# Geometry / filtering
NEAR_M = 0.35          # closer than this the D555 sees no depth and the thing overfills
MIN_DEPTH_M, MAX_DEPTH_M = 0.3, 6.0
FLOOR_Z = 0.04         # m in odom: a lock at or under this is a floor thing (= pixel_to_goal)
SIGMA0_M = 0.30        # how unsure a fresh lock is (VLM y error, depth holes)
SIGMA_MIN_M = 0.03
SIGMA_SHRINK = 0.7     # per good sighting
SIGMA_GROW_M_S = 0.15  # per second unseen: a moved thing (30 cm) is back in the window in ~2 s
SIGMA_MAX_M = 0.8
ALPHA = 0.35           # how far one sighting moves the stored point
GATE_MIN_M = 0.25
LOST_S = 3.0           # in view and unseen this long -> "lost"
OCCLUDED_LOST_S = 8.0  # ... longer when something is visibly in front of it
OCCLUDER_M = 0.30      # depth this much nearer than the point at its pixel = occluded


# ── geometry ─────────────────────────────────────────────────────────────────

def project(K, R, t, p):
    """odom point -> (u, v, z_cam). z_cam <= 0: behind the camera."""
    pc = R.T @ (np.asarray(p, float) - t)
    z = pc[2]
    if z <= 1e-6:
        return None, None, z
    return K[0, 0] * pc[0] / z + K[0, 2], K[1, 1] * pc[1] / z + K[1, 2], z


def ray(K, R, t, u, v):
    """Pixel -> (origin, unit direction) in odom."""
    d = np.array([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], 1.0])
    d = R @ d
    return t.copy(), d / np.linalg.norm(d)


def floor_hit(origin, direction, z0=0.0):
    """Where a ray meets the floor plane z = z0, or None (parallel / upward / behind)."""
    if direction[2] >= -1e-6:
        return None
    s = (z0 - origin[2]) / direction[2]
    return origin + s * direction if s > 0 else None


def point_at_depth(K, R, t, u, v, z_cam):
    """Pixel + optical-axis depth (what a RealSense pixel stores) -> odom point."""
    pc = np.array([(u - K[0, 2]) / K[0, 0] * z_cam, (v - K[1, 2]) / K[1, 1] * z_cam, z_cam])
    return R @ pc + t


def standoff_goal(obj_xy, robot_xy, standoff):
    """(x, y, yaw): `standoff` short of the object on the robot->object line, facing it.
    The same rule as pixel_to_goal's goal (its _STANDOFF_M, 0.45 m)."""
    ox, oy = obj_xy
    dx, dy = ox - robot_xy[0], oy - robot_xy[1]
    d = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    if d <= standoff:
        return robot_xy[0], robot_xy[1], yaw
    return ox - standoff * dx / d, oy - standoff * dy / d, yaw


def goal_update_due(sent_obj_xy, obj_xy, robot_xy, since_sent_s,
                    min_shift_m=0.12, min_interval_s=2.0, hands_off_m=0.8):
    """Should an approach re-send its goal for the refined point? /reach restarts its
    attempts on every new goal, so only for a real shift, not too often, and never
    in the last metre (goal_exec is finishing on the stored point there)."""
    if since_sent_s < min_interval_s:
        return False
    if math.hypot(obj_xy[0] - robot_xy[0], obj_xy[1] - robot_xy[1]) < hands_off_m:
        return False
    return math.hypot(obj_xy[0] - sent_obj_xy[0], obj_xy[1] - sent_obj_xy[1]) >= min_shift_m


# ── the lock ─────────────────────────────────────────────────────────────────

def _visible(u, v, tw, th, w, h):
    """Share of the predicted box inside the image (a cut-off look cannot match)."""
    if tw <= 0 or th <= 0:
        return 0.0
    ix = max(0.0, min(w, u + tw / 2) - max(0.0, u - tw / 2))
    iy = max(0.0, min(h, v + th / 2) - max(0.0, v - th / 2))
    return ix * iy / (tw * th)


def _elevation(cam_t, p):
    """sin of the angle the camera looks DOWN at p (floor foreshortening)."""
    d = np.asarray(p, float) - cam_t
    return max(1e-3, float(-d[2] / np.linalg.norm(d)))


def _grey(img):
    return img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


class Target:
    def __init__(self, lock_img, box, point, cam, K, now, label="", floor=None):
        """lock_img: the frame the thing was found in (BGR); box: (x0, y0, x1, y1) in
        it; point: its odom point (pixel_to_goal's "object"); cam: (R, t) of that
        frame; floor: a floor thing (default: point z <= FLOOR_Z)."""
        R, t = cam
        self.label = label
        self.p = np.asarray(point, float).copy()
        self.p_lock = self.p.copy()
        self.floor = bool(self.p[2] <= FLOOR_Z) if floor is None else floor
        g = _grey(lock_img)
        x0, y0, x1, y1 = box
        # A small box (a dot on the floor seen at a shallow angle is ~8 px tall)
        # gets context around it. For a floor thing that context is floor --
        # rigid with the dot, so it helps the match rather than hurting it.
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        bw, bh = max(x1 - x0, MIN_BOX_PX), max(y1 - y0, MIN_BOX_PX)
        x0, y0, x1, y1 = (int(round(c)) for c in (cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2))
        x0, y0 = max(0, x0), max(0, y0)
        x1, y1 = min(g.shape[1], x1), min(g.shape[0], y1)
        if x1 - x0 < TEMPLATE_MIN_PX or y1 - y0 < TEMPLATE_MIN_PX:
            raise ValueError(f"box {box} is too small to track (< {TEMPLATE_MIN_PX} px)")
        crop = g[y0:y1, x0:x1]
        shrink = min(1.0, TEMPLATE_MAX_PX / max(crop.shape))
        self.t0 = cv2.resize(crop, None, fx=shrink, fy=shrink, interpolation=cv2.INTER_AREA) if shrink < 1 else crop.copy()
        self.t_adapt = self.t0.astype(np.float32)
        _, _, z = project(K, R, t, self.p)
        # what the template's pixel size means: at range z_lock it is t0 * (1/shrink) px
        self.z_lock = max(z, 0.05)
        self.elev_lock = _elevation(t, self.p)
        self.px_per_t = 1.0 / shrink
        self.sigma = SIGMA0_M
        self.last_seen = now
        self.last_t = now
        self.seen = 0
        self.rejected = 0
        self.state = "tracking"
        self.score = 1.0
        self.pixel = (0.5 * (x0 + x1), 0.5 * (y0 + y1))
        self.depth_src = "lock"

    # one frame ---------------------------------------------------------------
    def update(self, img, depth, cam, K, now):
        """img: BGR or grey; depth: metres (float) aligned to img, 0/NaN = none, or None."""
        R, t = cam
        dt = max(0.0, now - self.last_t)
        self.last_t = now
        h, w = img.shape[:2]
        u, v, z = project(K, R, t, self.p)
        scale = self.z_lock / z if z and z > 0 else 0
        th, tw = (np.array(self.t0.shape[:2]) * self.px_per_t * scale) if scale else (0, 0)
        if self.floor and scale:
            # a flat thing on the floor grows TALLER faster than wider as the
            # camera nears it (foreshortening): height also goes with sin(elevation)
            th *= _elevation(t, self.p) / max(self.elev_lock, 1e-3)

        if z is None or z <= 0 or u is None:
            return self._unseen("out_of_view", dt, now, grow=0.5)
        if z < NEAR_M or tw > w * 0.9 or th > h * 0.9:
            return self._unseen("too_close", dt, now, grow=0.0)
        if _visible(u, v, tw, th, w, h) < MIN_VISIBLE:
            return self._unseen("out_of_view", dt, now, grow=0.5)
        if min(tw, th) < TEMPLATE_MIN_PX:
            return self._unseen("coasting", dt, now)

        g = _grey(img)
        f = K[0, 0]
        pad = int(np.clip(f * self.sigma / z + max(tw, th) / 2, SEARCH_MIN_PX, SEARCH_MAX_PX))
        found = self._match(g, u, v, tw, th, pad)
        if found is None:
            occluded = self._occluded(depth, u, v, z, tw, th)
            return self._unseen("occluded" if occluded else "coasting", dt, now)
        uc, vc, score = found

        meas, src = self._measure(depth, cam, K, uc, vc, tw, th, z)
        if meas is None:
            return self._unseen("coasting", dt, now)

        if np.linalg.norm(meas - self.p) > max(GATE_MIN_M, 3 * self.sigma):
            self.rejected += 1
            return self._unseen("coasting", dt, now)
        self.p = self.p + ALPHA * (meas - self.p)
        self.sigma = max(SIGMA_MIN_M, self.sigma * SIGMA_SHRINK)
        return self._seen(uc, vc, score, src, now, g, tw, th)

    # steps -------------------------------------------------------------------
    def _templates(self, tw, th):
        size = (max(1, int(round(tw))), max(1, int(round(th))))
        a = cv2.resize(self.t0, size, interpolation=cv2.INTER_AREA if tw < self.t0.shape[1] else cv2.INTER_LINEAR)
        b = cv2.resize(self.t_adapt.astype(np.uint8), size, interpolation=cv2.INTER_LINEAR)
        return a, b

    def _match(self, g, u, v, tw, th, pad):
        h, w = g.shape
        x0, y0 = int(max(0, u - pad)), int(max(0, v - pad))
        x1, y1 = int(min(w, u + pad + 1)), int(min(h, v + pad + 1))
        win = g[y0:y1, x0:x1]
        a, b = self._templates(tw, th)
        if win.shape[0] < a.shape[0] + 2 or win.shape[1] < a.shape[1] + 2:
            return None
        res = np.maximum(cv2.matchTemplate(win, a, cv2.TM_CCOEFF_NORMED),
                         cv2.matchTemplate(win, b, cv2.TM_CCOEFF_NORMED))
        res = np.nan_to_num(res, nan=-1.0)
        # centre of each candidate position, in image pixels
        ys, xs = np.mgrid[0:res.shape[0], 0:res.shape[1]]
        cx = xs + x0 + a.shape[1] / 2.0
        cy = ys + y0 + a.shape[0] / 2.0
        d = np.hypot(cx - u, cy - v) / max(pad, 1)
        weighted = res - DIST_PENALTY * d * d
        i = np.unravel_index(np.argmax(weighted), weighted.shape)
        best_w, best_raw = weighted[i], res[i]
        if best_raw < ACCEPT:
            return None
        # ambiguity: the best weighted score outside this peak's neighbourhood
        sup = weighted.copy()
        ry, rx = max(1, a.shape[0] // 2), max(1, a.shape[1] // 2)
        sup[max(0, i[0] - ry):i[0] + ry + 1, max(0, i[1] - rx):i[1] + rx + 1] = -9
        if sup.max() > best_w - AMBIG_MARGIN and res[np.unravel_index(np.argmax(sup), sup.shape)] >= ACCEPT:
            return None
        # sub-pixel peak (parabola through the neighbours)
        py, px = i
        def sub(c, l, r):
            den = l - 2 * c + r
            return 0.0 if abs(den) < 1e-9 else 0.5 * (l - r) / den
        ox = sub(res[py, px], res[py, px - 1], res[py, px + 1]) if 0 < px < res.shape[1] - 1 else 0.0
        oy = sub(res[py, px], res[py - 1, px], res[py + 1, px]) if 0 < py < res.shape[0] - 1 else 0.0
        return cx[i] + ox, cy[i] + oy, float(best_raw)

    def _depth_patch(self, depth, uc, vc, tw, th, frac=0.5):
        if depth is None:
            return None
        h, w = depth.shape[:2]
        rx, ry = max(1, int(tw * frac / 2)), max(1, int(th * frac / 2))
        p = depth[max(0, int(vc) - ry):min(h, int(vc) + ry + 1), max(0, int(uc) - rx):min(w, int(uc) + rx + 1)]
        p = p[np.isfinite(p)]
        return p[(p > MIN_DEPTH_M) & (p < MAX_DEPTH_M)]

    def _measure(self, depth, cam, K, uc, vc, tw, th, z_pred):
        R, t = cam
        if self.floor:
            o, d = ray(K, R, t, uc, vc)
            hit = floor_hit(o, d, z0=self.p_lock[2])
            if hit is not None:
                return hit, "floor"
        vals = self._depth_patch(depth, uc, vc, tw, th)
        if vals is not None and vals.size >= 5:
            near = vals[np.abs(vals - z_pred) < max(0.25, 0.2 * z_pred)]
            # a thin thing's box is mostly background: prefer depths near the prediction
            zc = float(np.median(near)) if near.size >= max(5, 0.15 * vals.size) else float(np.median(vals))
            return point_at_depth(K, R, t, uc, vc, zc), "depth"
        # bearing only: keep the range, take the new direction
        o, d = ray(K, R, t, uc, vc)
        return o + d * float(np.linalg.norm(self.p - o)), "bearing"

    def _occluded(self, depth, u, v, z, tw, th):
        vals = self._depth_patch(depth, u, v, tw, th, frac=0.6)
        return vals is not None and vals.size >= 5 and float(np.median(vals)) < z - OCCLUDER_M

    def _seen(self, uc, vc, score, src, now, g, tw, th):
        self.state, self.score, self.pixel, self.depth_src = "tracking", score, (uc, vc), src
        self.last_seen = now
        self.seen += 1
        if score >= STRONG:   # refresh the look slowly (viewpoint changes), at lock scale
            h, w = g.shape
            x0, y0 = int(round(uc - tw / 2)), int(round(vc - th / 2))
            x1, y1 = x0 + int(round(tw)), y0 + int(round(th))
            if x0 >= 0 and y0 >= 0 and x1 <= w and y1 <= h and x1 > x0 and y1 > y0:
                crop = cv2.resize(g[y0:y1, x0:x1], (self.t0.shape[1], self.t0.shape[0]), interpolation=cv2.INTER_AREA)
                self.t_adapt = (1 - ADAPT) * self.t_adapt + ADAPT * crop.astype(np.float32)
        return self.status(now)

    def _unseen(self, why, dt, now, grow=1.0):
        self.sigma = min(SIGMA_MAX_M, self.sigma + grow * SIGMA_GROW_M_S * dt)
        self.score = 0.0
        unseen_s = now - self.last_seen
        if why in ("coasting", "occluded") and unseen_s > (OCCLUDED_LOST_S if why == "occluded" else LOST_S):
            why = "lost"
        self.state = why
        return self.status(now)

    def status(self, now):
        return {
            "state": self.state,
            "label": self.label,
            "object": [round(float(c), 4) for c in self.p],
            "sigma_m": round(self.sigma, 3),
            "score": round(self.score, 3),
            "pixel": [round(float(c), 1) for c in self.pixel] if self.state == "tracking" else None,
            "depth": self.depth_src if self.state == "tracking" else None,
            "unseen_s": round(now - self.last_seen, 2),
            "sightings": self.seen,
            "from_lock_m": round(float(np.linalg.norm(self.p - self.p_lock)), 3),
            "rejected": self.rejected,
            "floor": self.floor,
        }
