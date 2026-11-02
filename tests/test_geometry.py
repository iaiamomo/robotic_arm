import numpy as np
import pytest

from braccio_ns.calibration import CameraToRobot, default_reference_points, umeyama
from braccio_ns.kinematics import BraccioIK, Unreachable
from braccio_ns.world import Grid


def rot(axis, a):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K


def test_umeyama_recovers_similarity():
    rng = np.random.default_rng(1)
    R, t, s = rot([1, 2, 3], 0.7), np.array([0.5, -0.2, 0.3]), 1.03
    src = rng.uniform(-0.3, 0.3, (33, 3))
    dst = (s * R @ src.T).T + t
    R2, t2, s2 = umeyama(src, dst)
    assert np.allclose(R2, R, atol=1e-9) and np.allclose(t2, t, atol=1e-9) and abs(s2 - s) < 1e-9


def test_calibration_fit_with_noise(cfg, tmp_path):
    rng = np.random.default_rng(2)
    rob = np.asarray(default_reference_points(cfg["grid"], 33))
    assert len(rob) == 33
    R, t = rot([0, 1, 0.3], 2.2), np.array([0.6, 0.0, 0.35])
    cam = (R.T @ (rob - t).T).T + rng.normal(0, 0.002, rob.shape)
    tf = CameraToRobot.fit(cam, rob)
    assert tf.rmse < 0.004
    tf.save(tmp_path / "cal.json")
    tf2 = CameraToRobot.load(tmp_path / "cal.json")
    assert np.allclose(tf2(cam), tf(cam))


def test_ik_reaches_every_cell_and_matches_fk(cfg):
    ik, g = BraccioIK(cfg["robot"]), Grid(cfg["grid"])
    cube = cfg["vision"]["cube_size_m"]
    for cell in g.cells():
        x, y = g.center(cell)
        for z in (cube / 2, 1.5 * cube, 1.5 * cube + cfg["robot"]["approach_height_m"]):
            sol = ik.solve([x, y, z])
            fw = ik.q_to_firmware(sol["q"], cfg["robot"]["grip_closed_deg"])
            assert not ik.check_firmware_limits(fw)
            # round trip through firmware degrees
            assert np.linalg.norm(ik.fk_firmware(fw)[:3, 3] - [x, y, z]) < 1e-6


def test_ik_rejects_out_of_reach(cfg):
    ik = BraccioIK(cfg["robot"])
    with pytest.raises(Unreachable):
        ik.solve([0.8, 0.0, 0.0])
    with pytest.raises(Unreachable):
        ik.solve([0.05, 0.0, 0.0])  # too close: elbow/wrist limits of the firmware
