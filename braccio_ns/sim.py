"""Hardware-free simulation: a synthetic RealSense (colour + depth rendering of
cube top faces) and a simulated arm that executes MOVE commands through FK and
grasps/releases cubes. Used by `python -m braccio_ns --sim` and by the tests.
"""

import cv2
import numpy as np

from .calibration import CameraToRobot
from .camera import PinholeIntrinsics, deproject_pinhole

BGR = {"RED": (40, 40, 210), "GREEN": (50, 190, 50), "BLUE": (210, 60, 40), "MYSTERY": (200, 200, 200)}


class SimScene:
    def __init__(self, cube=0.03):
        self.cube = cube
        self.cubes = []  # dicts: color, xyz (centre)

    def add(self, color, x, y, level=1):
        self.cubes.append({"color": color, "xyz": np.array([x, y, (level - 0.5) * self.cube])})

    def top_at(self, xy, tol, exclude=None):
        """Highest cube whose centre is within tol (xy) — the one the gripper can reach."""
        c = [k for k in self.cubes if k is not exclude and np.linalg.norm(k["xyz"][:2] - xy) < tol]
        return max(c, key=lambda k: k["xyz"][2], default=None)


def look_at(eye, target):
    """Camera optical frame (x right, y down, z forward) expressed in robot frame."""
    z = np.asarray(target, float) - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, [0, 0, 1.0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.stack([x, y, z], axis=1)  # columns


class SimCamera:
    def __init__(self, scene, width=640, height=480, eye=(0.58, 0.0, 0.30), target=(0.30, 0.0, 0.0), noise=2.0, seed=0):
        self.scene, self.w, self.h = scene, width, height
        self.intr = PinholeIntrinsics(615.0, 615.0, width / 2, height / 2, width, height)
        self.C = np.asarray(eye, float)
        self.R = look_at(self.C, target)  # cam -> robot rotation
        self.depth_scale = 0.001
        self.noise = noise
        self.rng = np.random.default_rng(seed)
        self.true_cam_to_robot = CameraToRobot(self.R, self.C, 1.0)

    def to_cam(self, p):
        return self.R.T @ (np.asarray(p) - self.C)

    def project(self, p):
        pc = self.to_cam(p)
        return np.array(
            [self.intr.fx * pc[0] / pc[2] + self.intr.ppx, self.intr.fy * pc[1] / pc[2] + self.intr.ppy]
        ), pc[2]

    def _table_depth(self):
        u, v = np.meshgrid(np.arange(self.w), np.arange(self.h))
        rays = (
            np.stack(
                [(u - self.intr.ppx) / self.intr.fx, (v - self.intr.ppy) / self.intr.fy, np.ones_like(u, float)], -1
            )
            @ self.R.T
        )  # robot-frame rays with z_cam = 1
        t = -self.C[2] / np.where(np.abs(rays[..., 2]) < 1e-9, -1e-9, rays[..., 2])
        return np.where(t > 0, t, 0)  # = z_cam depth since z_cam component is 1

    def get_frames(self):
        color = np.full((self.h, self.w, 3), 18, np.uint8)
        depth = self._table_depth()
        half = self.scene.cube / 2
        order = sorted(self.scene.cubes, key=lambda k: -np.linalg.norm(k["xyz"] - self.C))  # far first
        for k in order:
            cx, cy, cz = k["xyz"]
            top = cz + half
            corners = [(cx - half, cy - half), (cx + half, cy - half), (cx + half, cy + half), (cx - half, cy + half)]
            # front (camera-facing, +x) side face, drawn first so the top face overwrites the shared edge
            side = [
                (cx + half, cy - half, top),
                (cx + half, cy + half, top),
                (cx + half, cy + half, cz - half),
                (cx + half, cy - half, cz - half),
            ]
            for face, shade in ((side, 0.8), ([(x, y, top) for x, y in corners], 1.0)):
                pts, zs = zip(*(self.project(p) for p in face), strict=True)
                poly = np.round(np.array(pts)).astype(np.int32)
                col = tuple(int(c * shade) for c in BGR[k["color"]])
                cv2.fillConvexPoly(color, poly, col)
                m = np.zeros((self.h, self.w), np.uint8)
                cv2.fillConvexPoly(m, poly, 1)
                depth[m == 1] = np.mean(zs)
        if self.noise:
            color = np.clip(color + self.rng.normal(0, self.noise, color.shape), 0, 255).astype(np.uint8)
        return {
            "color": color,
            "depth": (depth / self.depth_scale).astype(np.uint16),
            "intrinsics": self.intr,
            "depth_scale": self.depth_scale,
        }

    def deproject(self, frame, uv, patch=5):
        u, v = int(round(uv[0])), int(round(uv[1]))
        win = frame["depth"][max(v - patch, 0) : v + patch + 1, max(u - patch, 0) : u + patch + 1].astype(float)
        win = win[win > 0]
        if win.size == 0:
            return None
        return deproject_pinhole(self.intr, (u, v), float(np.median(win)) * self.depth_scale)

    def close(self):
        pass


class SimArm:
    """Interprets MOVE commands with FK; grasps the top cube under the tip when the
    gripper closes, drops it onto whatever is below when it opens."""

    def __init__(self, ik, scene, robot_cfg, grasp_tol=0.012):
        self.ik, self.scene, self.cfg, self.tol = ik, scene, robot_cfg, grasp_tol
        self.held, self.log, self.grip_deg = None, [], robot_cfg["home"][0]
        self.last_fw = list(robot_cfg["home"])
        self.closed_thr = (robot_cfg["grip_open_deg"] + robot_cfg["grip_closed_deg"]) / 2

    def _closed(self, g):
        return (g >= self.closed_thr) == (self.cfg["grip_closed_deg"] > self.cfg["grip_open_deg"])

    def move(self, fw):
        self.log.append("MOVE " + " ".join(f"{a:.1f}" for a in fw))
        tip = self.ik.fk_firmware(fw)[:3, 3]
        tip[2] -= self.ik.droop_at(np.hypot(tip[0], tip[1]))  # behave like the real, sagging arm
        was_closed, now_closed = self._closed(self.grip_deg), self._closed(fw[0])
        if self.held is not None:
            self.held["xyz"] = tip.copy()
        if now_closed and not was_closed:
            k = self.scene.top_at(tip[:2], self.tol)
            if k is not None and abs(k["xyz"][2] - tip[2]) < self.tol:
                self.held = k
        elif was_closed and not now_closed and self.held is not None:
            below = self.scene.top_at(tip[:2], self.scene.cube * 0.6, exclude=self.held)
            base = 0.0 if below is None else below["xyz"][2] + self.scene.cube / 2
            if below is not None:
                tip[:2] = below["xyz"][:2]  # cube self-centres on the one below
            self.held["xyz"] = np.array([tip[0], tip[1], base + self.scene.cube / 2])
            self.held = None
        self.grip_deg, self.last_fw = fw[0], list(fw)
        return "OK"

    def grip(self, g):
        """GRIP command: only the gripper moves, the other joints stay where they are."""
        n = len(self.log)
        self.move([g] + self.last_fw[1:])
        self.log[n] = f"GRIP {g:.1f}"
        return "OK"

    def home(self):
        self.log.append("HOME")
        self.last_fw = list(self.cfg["home"])
        if self.held is None:
            self.grip_deg = self.cfg["home"][0]
        return "OK"

    def close(self):
        pass
