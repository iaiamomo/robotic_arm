"""Perception (thesis §3.3): background subtraction + chromatic dominance,
3D localisation with the RealSense depth map, and a simple tracker that
assigns persistent IDs and fills `target_memory`.
"""

import threading

import cv2
import numpy as np

from .world import Grid


def deproject_pixels(intr, us, vs, depth_m):
    """Vectorised pinhole deprojection (pixel + depth -> camera optical frame).
    Uses fx, fy, ppx, ppy of rs.intrinsics; the D435 colour stream has ~zero distortion."""
    x = (us - intr.ppx) / intr.fx
    y = (vs - intr.ppy) / intr.fy
    return np.stack([x * depth_m, y * depth_m, depth_m], axis=1)


COLORS_BGR = {"RED": (0, 0, 255), "GREEN": (0, 200, 0), "BLUE": (255, 0, 0)}
FONT = cv2.FONT_HERSHEY_SIMPLEX


# ------------------------------------------------- thesis functions (Listings 3.6, 3.7)
def get_mask_from_frames(background, frame, threshold=50, mode="max"):
    """Listing 3.6. mode="gray" is the thesis version (grayscale of the difference);
    mode="max" thresholds the largest per-channel difference instead, which does not
    under-segment dark saturated colours (blue has low luminance)."""
    diff = cv2.absdiff(background, frame)
    diff_gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY) if mode == "gray" else diff.max(axis=2)
    mask = cv2.inRange(diff_gray, threshold, 255)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    return mask, diff


def classify_color(mean_bgr, margin=10):
    """Chromatic dominance: the dominant channel must exceed both others by `margin`.
    (The thesis listing applies no margin to GREEN; here it is symmetric.)"""
    b, g, r = mean_bgr
    if b > g + margin and b > r + margin:
        return "BLUE"
    if g > b + margin and g > r + margin:
        return "GREEN"
    if r > b + margin and r > g + margin:
        return "RED"
    return None


def color_masks(frame, margin=10):
    """Per-pixel chromatic dominance (same rule as classify_color, vectorised)."""
    b, g, r = [frame[..., i].astype(np.int16) for i in range(3)]
    return {
        "BLUE": (b > g + margin) & (b > r + margin),
        "GREEN": (g > b + margin) & (g > r + margin),
        "RED": (r > b + margin) & (r > g + margin),
    }


def identify_cubes(mask, frame, min_area=900, aspect=(0.8, 1.2), margin=10):
    """Listing 3.7 classifies the mean colour of each foreground blob. With a tilted
    camera a stack forms a single blob containing two colours, so here the
    foreground is first split by per-pixel colour and each colour component is
    filtered by area and aspect ratio."""
    results = []
    for label, cm in color_masks(frame, margin).items():
        m = cv2.morphologyEx((cm & (mask > 0)).astype(np.uint8) * 255, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)
            if area > min_area and aspect[0] < w / h < aspect[1]:
                mo = cv2.moments(contour)
                results.append(
                    {
                        "label": label,
                        "bbox": (x, y, w, h),
                        "center": (mo["m10"] / mo["m00"], mo["m01"] / mo["m00"]),
                        "contour": contour,
                    }
                )
    return results


def drop_occluded(dets, cube):
    """A component whose top is clearly lower than an xy-overlapping one is the
    visible side face of a covered cube: drop it (the cube becomes HIDDEN_UNDER_*)."""
    keep = []
    for d in dets:
        covered = any(
            o is not d
            and o["pos"][2] > d["pos"][2] + cube / 2
            and np.linalg.norm(np.subtract(o["pos"][:2], d["pos"][:2])) < cube
            for o in dets
        )
        if not covered:
            keep.append(d)
    return keep


# ------------------------------------------------- tracking
class Track:
    def __init__(self, tid, color, pos):
        self.id, self.color, self.pos = tid, color, np.asarray(pos, float)
        self.hits, self.misses = 1, 0


class Tracker:
    """Nearest-neighbour association per colour. Absorbs short detection drop-outs
    ("detection flickering", thesis §5) so IDs stay stable during a mission.
    A track under a detected cube (occluded) does not age. A track with colour
    None (a hidden block seeded after execution) matches any colour and adopts it
    when the block is uncovered."""

    def __init__(self, match_tol=0.03, min_hits=3, max_misses=15, alpha=0.5, cube=0.03):
        self.match_tol, self.min_hits, self.max_misses, self.alpha, self.cube = (
            match_tol,
            min_hits,
            max_misses,
            alpha,
            cube,
        )
        self.tracks, self.counter = {}, 0

    def update(self, detections):
        unmatched = set(self.tracks)
        for det in detections:
            cands = [
                t for t in self.tracks.values() if t.id in unmatched and (t.color == det["label"] or t.color is None)
            ]
            best = min(cands, key=lambda t: np.linalg.norm(t.pos - det["pos"]), default=None)
            if best is not None and np.linalg.norm(best.pos - det["pos"]) < self.match_tol:
                best.pos = (1 - self.alpha) * best.pos + self.alpha * np.asarray(det["pos"])
                best.color = best.color or det["label"]
                best.hits += 1
                best.misses = 0
                unmatched.discard(best.id)
            else:
                self.counter += 1
                tid = f"{det['label']}_{self.counter}"
                self.tracks[tid] = Track(tid, det["label"], det["pos"])
        for tid in unmatched:
            t = self.tracks[tid]
            occluded = any(
                np.linalg.norm(np.subtract(d["pos"][:2], t.pos[:2])) < self.cube
                and d["pos"][2] > t.pos[2] + self.cube / 2
                for d in detections
            )
            if occluded:
                continue
            t.misses += 1
            if t.misses > self.max_misses:
                del self.tracks[tid]

    def confirmed(self):
        return [t for t in self.tracks.values() if t.hits >= self.min_hits]

    def sync(self, ws):
        """After execution: move tracks to their new symbolic positions, seed hidden blocks."""
        for bid, b in ws.blocks.items():
            pos = np.array([b.xy[0], b.xy[1], ws.top_z(bid)])
            if bid in self.tracks:
                self.tracks[bid].pos, self.tracks[bid].misses = pos, 0
            else:
                t = Track(bid, None if b.color == "MYSTERY" else b.color, pos)
                t.hits = self.min_hits
                self.tracks[bid] = t

    def reset(self):
        self.tracks.clear()


def tracks_to_memory(tracks, cube, xy_tol):
    """Build target_memory: level from top-surface height, on_top_of from xy overlap."""
    mem = {}
    for t in tracks:
        mem[t.id] = {
            "color": t.color or "MYSTERY",
            "pos": t.pos.tolist(),
            "level": max(1, int(round(t.pos[2] / cube))),
            "on_top_of": None,
        }
    for oid, d in mem.items():
        if d["level"] < 2:
            continue
        below = [
            o
            for o, e in mem.items()
            if e["level"] == d["level"] - 1 and np.linalg.norm(np.subtract(e["pos"][:2], d["pos"][:2])) < xy_tol
        ]
        d["on_top_of"] = below[0] if below else f"HIDDEN_UNDER_{oid}"
    return mem


# ------------------------------------------------- perception loop
def project_to_image(intr, pts_cam):
    """Camera-frame points (N, 3) -> pixels (N, 2) with the pinhole model, plus their depth."""
    pts = np.asarray(pts_cam, float)
    z = pts[:, 2]
    uv = np.stack([intr.fx * pts[:, 0] / z + intr.ppx, intr.fy * pts[:, 1] / z + intr.ppy], axis=1)
    return uv, z


def draw_grid(img, grid, tf, intr, color=(230, 230, 230)):
    """Outline every grid cell on the table plane (z = 0, robot frame) and label it r_c."""
    hr, hc = grid.row_step / 2, grid.col_step / 2
    for cell in grid.cells():
        c = grid.center(cell)
        xy = [c + a * hr + b * hc for a, b in ((-1, -1), (-1, 1), (1, 1), (1, -1))] + [c]
        uv, z = project_to_image(intr, tf.to_camera([[x, y, 0.0] for x, y in xy]))
        if (z <= 0).any():
            continue
        cv2.polylines(img, [np.round(uv[:4]).astype(np.int32)], True, color, 1, cv2.LINE_AA)
        label = cell.removeprefix("CELL_")
        (tw, th), _ = cv2.getTextSize(label, FONT, 0.35, 1)
        cv2.putText(img, label, (int(uv[4, 0] - tw / 2), int(uv[4, 1] + th / 2)), FONT, 0.35, color, 1, cv2.LINE_AA)


class Perception:
    def __init__(self, cfg, camera, cam_to_robot):
        self.v = cfg["vision"]
        self.camera, self.tf = camera, cam_to_robot
        self.background = None
        self.tracker = Tracker(
            self.v["track_match_tol_m"],
            self.v["track_min_hits"],
            self.v["track_max_misses"],
            cube=self.v["cube_size_m"],
        )
        self.grid = Grid(cfg["grid"])
        self.lock = threading.Lock()
        self.last_frame, self.last_dets, self.last_intr = None, [], None

    def update_background(self, n=10):
        """Average n frames of the empty workspace (GUI 'Update background')."""
        acc = None
        for _ in range(n):
            c = self.camera.get_frames()["color"].astype(np.float32)
            acc = c if acc is None else acc + c
        with self.lock:
            self.background = (acc / n).astype(np.uint8)
            self.tracker.reset()

    def step(self):
        frame = self.camera.get_frames()
        if self.background is None:
            with self.lock:  # keep the live preview running before the background is captured
                self.last_frame, self.last_dets, self.last_intr = frame["color"], [], frame["intrinsics"]
            return []
        mask, _ = get_mask_from_frames(
            self.background, frame["color"], self.v["diff_threshold"], self.v.get("diff_mode", "max")
        )
        cubes = identify_cubes(
            mask,
            frame["color"],
            self.v["min_area_px"],
            (self.v["aspect_min"], self.v["aspect_max"]),
            self.v["color_margin"],
        )
        dets = []
        for c in cubes:
            pos = self.localize(frame, c["contour"])
            if pos is not None:
                dets.append({**c, "pos": pos})
        dets = drop_occluded(dets, self.v["cube_size_m"])
        with self.lock:
            self.tracker.update(dets)
            self.last_frame, self.last_dets, self.last_intr = frame["color"], dets, frame["intrinsics"]
        return dets

    def localize(self, frame, contour, top_band=0.006):
        """3D position of a cube as [x, y, z_top] in the robot frame.

        With the camera tilted ~45 deg the blob contains the top face and the
        camera-facing side, so the 2D centroid lies near the front edge. We
        deproject every blob pixel, keep the points within `top_band` of the
        highest ones (the top face) and take their mean.
        """
        mask = np.zeros(frame["depth"].shape, np.uint8)
        cv2.drawContours(mask, [contour], -1, 1, -1)
        mask = cv2.erode(mask, np.ones((3, 3), np.uint8))  # drop mixed-depth border pixels
        vs, us = np.nonzero(mask)
        d = frame["depth"][vs, us].astype(float) * frame["depth_scale"]
        ok = d > 0
        if ok.sum() < 20:
            return None
        pc = deproject_pixels(frame["intrinsics"], us[ok], vs[ok], d[ok])
        pr = self.tf(pc)
        z_top = np.percentile(pr[:, 2], 95)
        top = pr[pr[:, 2] > z_top - top_band]
        return np.array([top[:, 0].mean(), top[:, 1].mean(), z_top])

    def sync(self, ws):
        with self.lock:
            self.tracker.sync(ws)

    def set_transform(self, cam_to_robot):
        """Use a new camera->robot calibration; tracks are reset since every position changes."""
        with self.lock:
            self.tf = cam_to_robot
            self.tracker.reset()

    def target_memory(self):
        with self.lock:
            return tracks_to_memory(self.tracker.confirmed(), self.v["cube_size_m"], self.v["stack_xy_tol_m"])

    def overlay(self, show_grid=False):
        with self.lock:
            if self.last_frame is None:
                return None
            img = self.last_frame.copy()
            if show_grid:
                if self.tf.is_identity:
                    msg = "Grid: no calibration, run calibrate"
                    y = img.shape[0] - 12
                    cv2.putText(img, msg, (10, y), FONT, 0.5, (0, 0, 0), 3)
                    cv2.putText(img, msg, (10, y), FONT, 0.5, (0, 255, 255), 1)
                else:
                    draw_grid(img, self.grid, self.tf, self.last_intr)
            if self.background is None:
                msg = "No background: press Update Background"
                cv2.putText(img, msg, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
                cv2.putText(img, msg, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
            tracks = {t.id: t for t in self.tracker.confirmed()}
            for d in self.last_dets:
                x, y, w, h = d["bbox"]
                tid = min(tracks.values(), key=lambda t: np.linalg.norm(t.pos - d["pos"]), default=None)
                cv2.rectangle(img, (x, y), (x + w, y + h), COLORS_BGR[d["label"]], 2)
                if tid is not None:
                    cv2.putText(img, tid.id, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            return img
