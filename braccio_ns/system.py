"""Wires the components together for real hardware or simulation."""

import threading
import time

import numpy as np

from .calibration import CameraToRobot
from .config import resolve
from .executor import Executor
from .kinematics import BraccioIK
from .llm import LLMGoalTranslator
from .mission import Mission
from .pddl import GoalError, extract_goal, validate_goal
from .vision import Perception
from .world import Grid


class FixedGoalTranslator:
    """Bypasses the LLM with a hand-written goal (testing planner/robot in isolation)."""

    def __init__(self, goal):
        self.goal = goal

    def translate(self, request, ws, grid, log=print):
        goal, atoms = validate_goal(extract_goal(self.goal), set(ws.blocks), set(grid.cells()))
        return goal, atoms, [self.goal]


def default_sim_scene(cfg):
    from .sim import SimScene

    s = SimScene(cfg["vision"]["cube_size_m"])
    g = Grid(cfg["grid"])
    for color, cell, level in [
        ("RED", "CELL_0_0", 1),
        ("GREEN", "CELL_1_2", 1),
        ("BLUE", "CELL_0_4", 1),
        ("BLUE", "CELL_2_4", 1),
        ("RED", "CELL_2_4", 2),
    ]:
        x, y = g.center(cell)
        s.add(color, x, y, level)
    return s


class System:
    def __init__(self, cfg, sim=False, goal=None, log=print, scene=None):
        self.cfg, self.sim, self.log = cfg, sim, log
        self.grid = Grid(cfg["grid"])
        self.ik = BraccioIK(cfg["robot"])
        if sim:
            from .sim import SimArm, SimCamera, SimScene

            self.scene = scene or default_sim_scene(cfg)
            self.camera = SimCamera(SimScene(self.scene.cube))  # empty scene for the background
            self.arm = SimArm(self.ik, self.scene, cfg["robot"])
            tf = self.camera.true_cam_to_robot
        else:
            from .camera import RealSenseCamera
            from .serial_link import BraccioSerial

            c, s = cfg["camera"], cfg["serial"]
            self.camera = RealSenseCamera(c["width"], c["height"], c["fps"])
            self.arm = BraccioSerial(s["port"], s["baudrate"], s["timeout_s"], s["move_settle_s"])
            cal = resolve(cfg["calibration"]["file"])
            if cal.exists():
                tf = CameraToRobot.load(cal)
                log(f"Loaded calibration (RMSE {tf.rmse * 1000:.1f} mm)" if tf.rmse else "Loaded calibration")
            else:
                log("WARNING: no calibration file, run `python -m braccio_ns calibrate` first")
                tf = CameraToRobot()
        self.perception = Perception(cfg, self.camera, tf)
        self.translator = (
            FixedGoalTranslator(goal) if goal else LLMGoalTranslator(cfg["llm"], cfg["planner"]["domain_file"])
        )
        self.executor = Executor(cfg, self.ik, self.arm, self.grid, log)
        self.mission = Mission(cfg, self.translator, self.executor, log)
        self._stop = threading.Event()
        self._camera_lock = threading.Lock()  # one camera reader at a time (vision loop / background / calibration)
        self._vision_thread = None

    def capture_background(self):
        with self._camera_lock:
            self.perception.update_background()
        if self.sim:
            self.camera.scene = self.scene  # cubes "placed" after the empty background

    def warmup(self, n=10):
        for _ in range(n):
            self.perception.step()

    def run_mission(self, request, execute=True):
        """Snapshot perception, run start_mission, then re-sync the tracker with the
        new symbolic state so object IDs survive the manipulation."""
        res = self.mission.start_mission(request, self.perception.target_memory(), execute=execute)
        if res.world is not None:
            self.perception.sync(res.world)
        return res

    def calibrate(self):
        """Eye-to-hand calibration, applied immediately (no restart needed). The vision loop is
        paused meanwhile, since the calibration reads the camera itself. On hardware the result
        is saved to calibration.file and the arm ends at HOME with the hand open."""
        from .calibration import CameraToRobot, default_reference_points, run_calibration

        cal = self.cfg["calibration"]
        with self._camera_lock:
            if self.sim:
                # The ArUco marker is not rendered in simulation: fit on noisy synthetic pairs instead.
                heights = tuple(cal.get("tip_heights_m", (0.05, 0.09, 0.13)))
                rob = np.asarray(default_reference_points(self.cfg["grid"], cal["n_points"], heights))
                true = self.camera.true_cam_to_robot
                cam = true.to_camera(rob) + np.random.default_rng(0).normal(0, 0.002, rob.shape)
                tf = CameraToRobot.fit(cam, rob)
                self.log(f"Simulated calibration RMSE {tf.rmse * 1000:.2f} mm (not saved)")
            else:
                try:
                    tf = run_calibration(self.cfg, self.camera, self.arm, self.ik, log=self.log)
                finally:
                    try:
                        self.executor.home_open()
                    except Exception as e:
                        self.log(f"HOME after calibration failed: {e}")
        self.perception.set_transform(tf)
        return tf

    def start_vision(self, period_s=0.05):
        def loop():
            while not self._stop.is_set():
                try:
                    with self._camera_lock:
                        self.perception.step()
                except Exception as e:  # keep the loop alive
                    self.log(f"vision error: {e}")
                    time.sleep(0.5)
                time.sleep(period_s)

        self._vision_thread = threading.Thread(target=loop, daemon=True)
        self._vision_thread.start()

    def shutdown(self):
        self._stop.set()
        try:
            self.camera.close()
            self.arm.close()
        except Exception:
            pass


__all__ = ["System", "FixedGoalTranslator", "GoalError"]
