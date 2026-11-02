"""Inverse kinematics with IKPy (thesis §3.2.1, Listing 3.2) and the mapping
IK joint radians <-> firmware degrees.

Chain: base_link(origin) -> base_yaw -> shoulder -> elbow -> wrist_pitch -> wrist_roll -> tip(fixed)
Wrist roll is excluded from IK: for top-down grasps it only spins the gripper
about its own axis, so it is held at `wrist_roll_deg`.
"""

import numpy as np

from .config import resolve

# Firmware mechanical limits (Listing 3.1)
FW_LIMITS = {
    "grip": (120.0, 230.0),
    "roll": (0.0, 315.0),
    "pitch": (80.0, 230.0),
    "elbow": (75.0, 225.0),
    "shoulder": (60.0, 250.0),
    "base": (0.0, 315.0),
}

ACTIVE_MASK = [False, True, True, True, True, False, False]
IDX = {"base": 1, "shoulder": 2, "elbow": 3, "wrist_pitch": 4, "wrist_roll": 5}


class Unreachable(RuntimeError):
    pass


class BraccioIK:
    def __init__(self, robot_cfg):
        import warnings

        import ikpy.chain

        with warnings.catch_warnings():
            warnings.simplefilter("ignore")  # ikpy warns about the fixed tip link
            self.chain = ikpy.chain.Chain.from_urdf_file(
                str(resolve(robot_cfg["urdf"])),
                base_elements=["base_link"],
                active_links_mask=ACTIVE_MASK,
            )
        if len(self.chain.links) != len(ACTIVE_MASK):
            raise RuntimeError(f"URDF produced {len(self.chain.links)} links, expected {len(ACTIVE_MASK)}")
        self.cfg = robot_cfg
        self.offsets = robot_cfg["joint_offsets_deg"]
        self.signs = robot_cfg["joint_signs"]
        self.tol = robot_cfg.get("ik_tolerance_m", 0.01)

    # ---------- angle mapping ----------
    def _fw(self, joint, q_rad):
        return self.offsets[joint] + self.signs[joint] * np.degrees(q_rad)

    def _rad(self, joint, fw_deg):
        return np.radians((fw_deg - self.offsets[joint]) / self.signs[joint])

    def q_to_firmware(self, q, grip_deg):
        """IK vector -> [grip, roll, pitch, elbow, shoulder, base] (firmware order)."""
        return [
            float(grip_deg),
            float(self.cfg["wrist_roll_deg"]),
            float(self._fw("wrist_pitch", q[IDX["wrist_pitch"]])),
            float(self._fw("elbow", q[IDX["elbow"]])),
            float(self._fw("shoulder", q[IDX["shoulder"]])),
            float(self._fw("base", q[IDX["base"]])),
        ]

    def firmware_to_q(self, fw):
        q = np.zeros(len(self.chain.links))
        q[IDX["wrist_pitch"]] = self._rad("wrist_pitch", fw[2])
        q[IDX["elbow"]] = self._rad("elbow", fw[3])
        q[IDX["shoulder"]] = self._rad("shoulder", fw[4])
        q[IDX["base"]] = self._rad("base", fw[5])
        return q

    @staticmethod
    def check_firmware_limits(fw):
        names = ["grip", "roll", "pitch", "elbow", "shoulder", "base"]
        bad = [
            (n, round(a, 1))
            for n, a in zip(names, fw, strict=True)
            if not FW_LIMITS[n][0] - 1e-6 <= a <= FW_LIMITS[n][1] + 1e-6
        ]
        return bad

    # ---------- kinematics ----------
    def fk(self, q):
        return self.chain.forward_kinematics(q)

    def fk_firmware(self, fw):
        return self.fk(self.firmware_to_q(fw))

    # ---------- geometry read from the URDF (so IK and FK always agree) ----------
    def _geometry(self):
        links = {link.name: link for link in self.chain.links}
        z = lambda name: float(links[name].origin_translation[2])
        h = z("base_yaw") + z("shoulder")  # table -> shoulder axis
        return h, z("elbow"), z("wrist_pitch"), z("wrist_roll") + z("tip")

    def _analytic(self, r, zt, tilt_rad, elbow_sign):
        """Planar IK in the radial plane. Angles from vertical, positive = toward +r."""
        h, L1, L2, L3 = self._geometry()
        phi = np.pi - tilt_rad  # tip axis direction (pointing down, leaning out)
        wr, wz = r - L3 * np.sin(phi), zt - L3 * np.cos(phi) - h
        c2 = (wr * wr + wz * wz - L1 * L1 - L2 * L2) / (2 * L1 * L2)
        if abs(c2) > 1:
            return None
        t2 = elbow_sign * np.arccos(c2)
        t1 = np.arctan2(wr, wz) - np.arctan2(L2 * np.sin(t2), L1 + L2 * np.cos(t2))
        t3 = phi - t1 - t2
        return t1, t2, (t3 + np.pi) % (2 * np.pi) - np.pi

    def _within_bounds(self, q):
        for i in (IDX["base"], IDX["shoulder"], IDX["elbow"], IDX["wrist_pitch"]):
            lo, hi = self.chain.links[i].bounds
            if not lo - 1e-9 <= q[i] <= hi + 1e-9:
                return False
        return True

    # smallest |tilt| first; negative = gripper leaning back toward the base
    TILTS_DEG = tuple(sorted(range(-44, 72, 2), key=lambda t: (abs(t), -t)))

    def solve(self, xyz, max_tilt_deg=70):
        """IK for tip position `xyz` (robot frame, metres).

        The gripper should point down. With the Braccio++ link lengths a vertical
        grasp on the table is usually infeasible, so we search for the smallest
        tilt (in the radial plane) that reaches the target within joint
        limits. Each solution is verified with IKPy forward kinematics on the URDF.
        Raises Unreachable if none exists.
        """
        target = np.asarray(xyz, dtype=float)
        yaw = float(np.arctan2(target[1], target[0]))
        r = float(np.hypot(target[0], target[1]))
        for tilt in (t for t in self.TILTS_DEG if abs(t) <= max_tilt_deg):
            for elbow_sign in (+1, -1):
                sol = self._analytic(r, target[2], np.radians(tilt), elbow_sign)
                if sol is None:
                    continue
                q = np.zeros(len(self.chain.links))
                q[IDX["base"]] = yaw
                q[IDX["shoulder"]], q[IDX["elbow"]], q[IDX["wrist_pitch"]] = sol
                if not self._within_bounds(q):
                    continue
                pos_err = float(np.linalg.norm(self.fk(q)[:3, 3] - target))
                if pos_err > self.tol:
                    continue  # URDF not planar as assumed -> do not trust this solution
                fw = self.q_to_firmware(q, self.cfg["grip_open_deg"])
                if self.check_firmware_limits(fw):
                    continue
                return {"q": q, "pos_err_m": pos_err, "tilt_deg": float(tilt)}
        raise Unreachable(
            f"target {np.round(target, 3).tolist()} unreachable within joint limits (max tilt {max_tilt_deg} deg)"
        )
