"""Eye-to-hand calibration.

Procedure: the arm visits N reference poses (33 by default) computed with IK: the
grasp point (tip) is spread over the grid area at safe heights above the table, and the
marker sits wherever the gripper puts it.
An ArUco marker (DICT_4X4_50, id 0) is attached to the end-effector. For each
point the camera averages 20 frames: marker centre pixel -> depth -> 3D point in
the camera frame (rs2_deproject_pixel_to_point). The robot-side point is the FK
of the commanded joints plus the marker offset. Umeyama then estimates the
similarity transform  p_robot = s * R @ p_cam + t  that minimises the RMSE.
"""

import json
import time

import numpy as np

from .config import resolve


# ---------------------------------------------------------------- Umeyama
def umeyama(src, dst, with_scale=True):
    """Least-squares similarity transform dst ≈ s R src + t (Umeyama, 1991).

    src, dst: (N, 3) arrays of corresponding points. Returns (R, t, s).
    """
    src, dst = np.asarray(src, float), np.asarray(dst, float)
    if src.shape != dst.shape or src.shape[0] < 3:
        raise ValueError("need >= 3 paired 3D points")
    n = src.shape[0]
    mu_s, mu_d = src.mean(0), dst.mean(0)
    xs, xd = src - mu_s, dst - mu_d
    cov = xd.T @ xs / n
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1  # avoid reflections
    R = U @ S @ Vt
    var_s = (xs**2).sum() / n
    s = float(np.trace(np.diag(D) @ S) / var_s) if with_scale else 1.0
    t = mu_d - s * R @ mu_s
    return R, t, s


class CameraToRobot:
    def __init__(self, R=None, t=None, s=1.0, rmse=None):
        self.R = np.eye(3) if R is None else np.asarray(R, float)
        self.t = np.zeros(3) if t is None else np.asarray(t, float)
        self.s, self.rmse = float(s), rmse

    def __call__(self, p_cam):
        p = np.asarray(p_cam, float)
        return (self.s * (self.R @ p.T)).T + self.t

    def to_camera(self, p_robot):
        """Inverse transform: robot-frame points (N, 3) -> camera frame."""
        p = np.asarray(p_robot, float)
        return ((p - self.t) @ self.R) / self.s

    @property
    def is_identity(self):
        """True for the placeholder used when no calibration file exists."""
        return self.s == 1.0 and np.allclose(self.R, np.eye(3)) and np.allclose(self.t, 0)

    @classmethod
    def fit(cls, cam_pts, robot_pts, with_scale=True):
        R, t, s = umeyama(cam_pts, robot_pts, with_scale)
        tf = cls(R, t, s)
        tf.rmse = float(np.sqrt(np.mean(np.sum((tf(cam_pts) - np.asarray(robot_pts)) ** 2, axis=1))))
        return tf

    def save(self, path):
        resolve(path).write_text(
            json.dumps({"R": self.R.tolist(), "t": self.t.tolist(), "s": self.s, "rmse_m": self.rmse}, indent=2)
        )

    @classmethod
    def load(cls, path):
        d = json.loads(resolve(path).read_text())
        return cls(d["R"], d["t"], d["s"], d.get("rmse_m"))


# ---------------------------------------------------------------- routine
def default_reference_points(grid_cfg, n_points=33, heights=(0.06, 0.10, 0.14)):
    """Spread n_points over the working area (grid bounding box) at a few heights."""
    ox, oy = grid_cfg["origin_xy"]
    xs = np.linspace(ox, ox + grid_cfg["row_step_xy"][0] * (grid_cfg["rows"] - 1), 4)
    ys = np.linspace(oy, oy + grid_cfg["col_step_xy"][1] * (grid_cfg["cols"] - 1), 4)
    pts = [(x, y, z) for z in heights for x in xs for y in ys]
    idx = np.linspace(0, len(pts) - 1, min(n_points, len(pts))).round().astype(int)
    return [pts[i] for i in idx]


def detect_marker_center(detector, color, marker_id):
    corners, ids, _ = detector.detectMarkers(color)
    if ids is None:
        return None
    for c, i in zip(corners, ids.flatten(), strict=True):
        if int(i) == marker_id:
            return c[0].mean(axis=0)  # (u, v)
    return None


def run_calibration(cfg, camera, arm, ik, log=print):
    """Interactive-free calibration loop. Returns a fitted CameraToRobot."""
    import cv2.aruco as aruco

    cal = cfg["calibration"]
    detector = aruco.ArucoDetector(aruco.getPredefinedDictionary(aruco.DICT_4X4_50), aruco.DetectorParameters())
    offset = np.asarray(cal["marker_offset_tip_m"], float)
    cam_pts, rob_pts = [], []

    heights = tuple(cal.get("tip_heights_m", (0.05, 0.09, 0.13)))
    for k, target in enumerate(default_reference_points(cfg["grid"], cal["n_points"], heights)):
        # `target` is for the grasp point: the tip stays clear of the table. The marker offset is
        # in the tip frame, so the marker position is taken from FK below, not from `target`.
        try:
            sol = ik.solve(np.asarray(target))
        except Exception as e:
            log(f"[{k}] skip {target}: {e}")
            continue
        fw = ik.q_to_firmware(sol["q"], cfg["robot"]["grip_closed_deg"])
        arm.move(fw)
        time.sleep(0.5)

        T = ik.fk(sol["q"])
        p_robot = T[:3, 3] + T[:3, :3] @ offset
        # FK is the model, which IK raised by the measured droop: the real tip is at `target`.
        # The sag is close to a rotation about the shoulder, so the marker drops in proportion
        # to its distance from the base axis.
        reach = np.hypot(target[0], target[1])
        p_robot[2] -= ik.droop_at(reach) * np.hypot(p_robot[0], p_robot[1]) / reach

        samples = []
        for _ in range(cal["frames_per_point"]):
            f = camera.get_frames()
            uv = detect_marker_center(detector, f["color"], cal["marker_id"])
            if uv is None:
                continue
            p = camera.deproject(f, uv)
            if p is not None:
                samples.append(p)
        if len(samples) < cal["frames_per_point"] // 2:
            log(f"[{k}] marker seen in {len(samples)} frames only, skipped")
            continue
        p_cam = np.median(np.asarray(samples), axis=0)
        cam_pts.append(p_cam)
        rob_pts.append(p_robot)
        log(f"[{k}] cam={np.round(p_cam, 4).tolist()} robot={np.round(p_robot, 4).tolist()}")

    arm.home()
    if len(cam_pts) < 6:
        raise RuntimeError(f"only {len(cam_pts)} valid points, calibration aborted")
    tf = CameraToRobot.fit(np.asarray(cam_pts), np.asarray(rob_pts))
    tf.save(cal["file"])
    np.savez(
        str(resolve(cal["file"])).replace(".json", "_points.npz"), X_mat=np.asarray(cam_pts), Y_mat=np.asarray(rob_pts)
    )
    log(f"Calibration saved, RMSE = {tf.rmse * 1000:.1f} mm over {len(cam_pts)} points")
    return tf
