"""Wires the components together for real hardware or simulation."""
import threading
import time

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
    for color, cell, level in [("RED", "CELL_0_0", 1), ("GREEN", "CELL_1_2", 1), ("BLUE", "CELL_0_4", 1),
                               ("BLUE", "CELL_2_5", 1), ("RED", "CELL_2_5", 2)]:
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
            self.camera = SimCamera(SimScene(self.scene.cube))       # empty scene for the background
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
        self.translator = FixedGoalTranslator(goal) if goal else \
            LLMGoalTranslator(cfg["llm"], cfg["planner"]["domain_file"])
        self.executor = Executor(cfg, self.ik, self.arm, self.grid, log)
        self.mission = Mission(cfg, self.translator, self.executor, log)
        self._stop = threading.Event()
        self._vision_thread = None

    def capture_background(self):
        self.perception.update_background()
        if self.sim:
            self.camera.scene = self.scene     # cubes "placed" after the empty background

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

    def start_vision(self, period_s=0.05):
        def loop():
            while not self._stop.is_set():
                try:
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
