"""Shared I/O helpers for the vision lesson.

Sources: RealSenseSource (Intel RealSense D435/D435i) and ReplaySource (.npz).
Includes visualization, configuration, and camera-to-robot calibration helpers.

Examples:
    python realsense_rgb_depth.py
    python realsense_rgb_depth.py --replay snapshots
"""

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
sys.path.insert(0, str(REPO))  # Reuse braccio_ns (Grid and calibration).

FONT = cv2.FONT_HERSHEY_SIMPLEX


# ------------------------------------------------------------------ config
def load_cfg():
    return json.loads((REPO / "config" / "default.json").read_text())


def load_cam_to_robot(source):
    """Camera-to-robot transform (p_robot = s R p_cam + t) from eye-to-hand calibration."""
    from braccio_ns.calibration import CameraToRobot

    path = REPO / load_cfg()["calibration"]["file"]
    if path.exists():
        return CameraToRobot.load(path)
    return None


# ------------------------------------------------------------------ sources
class Intrinsics:
    """Pinhole parameters with the same fields as rs.intrinsics."""

    def __init__(self, fx, fy, ppx, ppy, width, height):
        self.fx, self.fy, self.ppx, self.ppy, self.width, self.height = fx, fy, ppx, ppy, width, height

    def __repr__(self):
        return f"{self.width}x{self.height} fx={self.fx:.1f} fy={self.fy:.1f} ppx={self.ppx:.1f} ppy={self.ppy:.1f}"


class RealSenseSource:
    """RealSense source with depth-to-color alignment, optional filters and IR stream."""

    name = "RealSense"

    def __init__(self, width=640, height=480, fps=30, align=True, ir=False):
        import pyrealsense2 as rs

        self.rs = rs
        self.pipeline = rs.pipeline()
        conf = rs.config()
        conf.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        conf.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.ir = ir
        if ir:
            conf.enable_stream(rs.stream.infrared, 1, width, height, rs.format.y8, fps)
        self.profile = self.pipeline.start(conf)
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        # Raw z16 * depth_scale = meters (typically one unit = 1 mm).
        self.depth_scale = self.depth_sensor.get_depth_scale()
        self.align = rs.align(rs.stream.color)  # Reproject depth into the RGB view.
        self.align_on = align
        self.filters_on = False
        self.filters = [rs.spatial_filter(), rs.temporal_filter(), rs.hole_filling_filter()]
        for _ in range(30):  # Allow auto-exposure to settle.
            self.pipeline.wait_for_frames()

    def get_frames(self):
        fs = self.pipeline.wait_for_frames()
        if self.align_on:
            fs = self.align.process(fs)
        color, depth = fs.get_color_frame(), fs.get_depth_frame()
        # Aligned depth uses RGB intrinsics; otherwise use depth-sensor intrinsics.
        prof = color.profile if self.align_on else depth.profile
        i = prof.as_video_stream_profile().intrinsics
        if self.filters_on:
            for f in self.filters:
                depth = f.process(depth)
        out = {
            "color": np.asanyarray(color.get_data()).copy(),
            "depth": np.asanyarray(depth.get_data()).copy(),
            "intrinsics": Intrinsics(i.fx, i.fy, i.ppx, i.ppy, i.width, i.height),
            "depth_scale": self.depth_scale,
        }
        if self.ir:
            out["ir"] = np.asanyarray(fs.get_infrared_frame(1).get_data()).copy()
        return out

    def toggle_emitter(self):
        opt = self.rs.option.emitter_enabled
        if not self.depth_sensor.supports(opt):
            return None
        v = 0.0 if self.depth_sensor.get_option(opt) > 0 else 1.0
        self.depth_sensor.set_option(opt, v)
        return bool(v)

    def toggle_scene(self):
        pass

    def close(self):
        self.pipeline.stop()


class ReplaySource:
    """Replay snapshots saved with s. Press n to advance to the next snapshot."""

    name = "Replay"

    def __init__(self, folder):
        self.files = sorted(Path(folder).glob("*.npz"))
        if not self.files:
            sys.exit(f"No .npz files in {folder}")
        self.idx = 0
        self._load()

    def _load(self):
        d = np.load(self.files[self.idx])
        self.frame = {
            "color": d["color"],
            "depth": d["depth"],
            "intrinsics": Intrinsics(*d["intr"]),
            "depth_scale": float(d["depth_scale"]),
        }
        self.depth_scale = self.frame["depth_scale"]
        print(f"[replay] {self.files[self.idx].name}")

    def get_frames(self):
        time.sleep(1 / 30)
        return {k: (v.copy() if isinstance(v, np.ndarray) else v) for k, v in self.frame.items()}

    def toggle_scene(self):  # In replay, c / n advance to the next snapshot.
        self.idx = (self.idx + 1) % len(self.files)
        self._load()

    def toggle_emitter(self):
        return None

    def close(self):
        pass


def make_parser(description):
    p = argparse.ArgumentParser(description=description)
    p.add_argument("--replay", metavar="DIR", help="replay .npz snapshots saved with 's'")
    return p


def open_source(args, **rs_kwargs):
    if args.replay:
        return ReplaySource(args.replay)
    try:
        return RealSenseSource(**rs_kwargs)
    except Exception as e:  # Camera not connected.
        sys.exit(f"RealSense unavailable ({e}).\nConnect the camera or use --replay DIR")


def save_snapshot(frame, folder=HERE / "snapshots", tag="frame"):
    folder = Path(folder)
    folder.mkdir(exist_ok=True)
    i = frame["intrinsics"]
    path = folder / f"{time.strftime('%Y%m%d_%H%M%S')}_{tag}.npz"
    np.savez_compressed(
        path,
        color=frame["color"],
        depth=frame["depth"],
        intr=np.array([i.fx, i.fy, i.ppx, i.ppy, i.width, i.height]),
        depth_scale=frame["depth_scale"],
    )
    print(f"saved {path}")
    return path


# ------------------------------------------------------------------ background
def capture_background(source, n=10):
    """Average n frames of the empty scene, as in Perception.update_background.
    Averaging reduces independent sensor noise approximately as 1/sqrt(n)."""
    acc = None
    for _ in range(n):
        c = source.get_frames()["color"].astype(np.float32)
        acc = c if acc is None else acc + c
    return (acc / n).astype(np.uint8)


# ------------------------------------------------------------------ visualization
def put_text(img, text, org, scale=0.5, color=(255, 255, 255)):
    """Draw text with a black outline for readability."""
    cv2.putText(img, text, org, FONT, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, org, FONT, scale, color, 1, cv2.LINE_AA)


def put_lines(img, lines, org=(10, 20), scale=0.5, color=(255, 255, 255), step=18):
    for k, line in enumerate(lines):
        put_text(img, line, (org[0], org[1] + k * step), scale, color)


def colorize_depth(depth_raw, depth_scale, near=0.2, far=1.5):
    """Convert z16 depth to BGR: near = red, far = blue, missing depth = black."""
    d = depth_raw.astype(np.float32) * depth_scale
    norm = np.clip((d - near) / (far - near), 0, 1)
    img = cv2.applyColorMap((255 * (1 - norm)).astype(np.uint8), cv2.COLORMAP_JET)
    img[depth_raw == 0] = 0
    return img


def to_bgr(img):
    if img.ndim == 2:
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img


class Mosaic:
    """Arrange images in a grid and map mouse events to original (panel, u, v) coordinates."""

    def __init__(self, window, cols=2, scale=1.0):
        self.window, self.cols, self.scale = window, cols, scale
        self.shape = None
        self.click = None  # (panel, u, v) of the last left click
        self.hover = None  # (panel, u, v) under the mouse
        cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(window, self._on_mouse)

    def _on_mouse(self, event, x, y, flags, param):
        if self.shape is None:
            return
        h, w = self.shape
        x, y = int(x / self.scale), int(y / self.scale)
        panel = (y // h) * self.cols + x // w
        pos = (panel, x % w, y % h)
        self.hover = pos
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click = pos

    def show(self, panels, titles=None):
        h, w = panels[0].shape[:2]
        self.shape = (h, w)
        tiles = []
        for k, p in enumerate(panels):
            t = to_bgr(p).copy()
            if t.shape[:2] != (h, w):
                t = cv2.resize(t, (w, h), interpolation=cv2.INTER_NEAREST)
            if titles:
                put_text(t, titles[k], (8, h - 10), 0.55, (0, 255, 255))
            tiles.append(t)
        while len(tiles) % self.cols:
            tiles.append(np.zeros_like(tiles[0]))
        rows = [np.hstack(tiles[r : r + self.cols]) for r in range(0, len(tiles), self.cols)]
        img = np.vstack(rows)
        if self.scale != 1.0:
            img = cv2.resize(img, None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_AREA)
        cv2.imshow(self.window, img)


def trackbar(window, name, value, maximum):
    cv2.createTrackbar(name, window, value, maximum, lambda _v: None)


def tb(window, name):
    return cv2.getTrackbarPos(name, window)
