"""Plan execution: symbolic MOVE actions -> pick-and-place trajectories -> serial MOVE commands.

Every action of the plan is checked for IK feasibility on a copy of the world
*before* the arm moves, so an unreachable placement aborts the mission instead
of failing half-way (a failure mode reported in the thesis, §4.3).
"""

import copy

import numpy as np

from .kinematics import Unreachable


class ExecutionError(RuntimeError):
    pass


class Executor:
    def __init__(self, cfg, ik, arm, grid, log=print):
        self.r = cfg["robot"]
        self.ik, self.arm, self.grid, self.log = ik, arm, grid, log

    # ---- primitives
    def _pose(self, xyz, grip):
        sol = self.ik.solve(xyz)
        return self.ik.q_to_firmware(sol["q"], grip)

    def pick_place_poses(self, ws, block, target):
        up = np.array([0, 0, self.r["approach_height_m"]])
        g_open, g_closed = self.r["grip_open_deg"], self.r["grip_closed_deg"]
        pick = ws.grasp_point(block)
        place = ws.place_point(target, self.grid)
        return [
            ("above pick", self._pose(pick + up, g_open)),
            ("descend", self._pose(pick, g_open)),
            ("grasp", self._pose(pick, g_closed)),
            ("lift", self._pose(pick + up, g_closed)),
            ("above place", self._pose(place + up, g_closed)),
            ("lower", self._pose(place, g_closed)),
            ("release", self._pose(place, g_open)),
            ("retract", self._pose(place + up, g_open)),
        ]

    # ---- checks
    def _check_action(self, ws, name, args):
        if name != "MOVE" or len(args) != 3:
            raise ExecutionError(f"unsupported action {name} {args}")
        b, frm, tgt = args
        if b not in ws.blocks:
            raise ExecutionError(f"unknown block {b}")
        if ws.blocks[b].support != frm:
            raise ExecutionError(f"plan/world mismatch: {b} is on {ws.blocks[b].support}, plan says {frm}")
        return b, frm, tgt

    def dry_run(self, ws, plan):
        """Simulate the plan symbolically and verify every pose is reachable. Returns all poses."""
        sim = copy.deepcopy(ws)
        all_poses = []
        for i, (name, args) in enumerate(plan):
            b, _, tgt = self._check_action(sim, name, args)
            try:
                all_poses.append(self.pick_place_poses(sim, b, tgt))
            except Unreachable as e:
                raise ExecutionError(f"step {i} {name} {' '.join(args)}: {e}") from e
            sim.apply_move(b, tgt, self.grid)
        return all_poses, sim

    # ---- execution
    def execute(self, ws, plan, stop_event=None):
        self.dry_run(ws, plan)
        for i, (name, args) in enumerate(plan):
            if stop_event is not None and stop_event.is_set():
                self.log("Mission stopped by user")
                break
            b, _, tgt = self._check_action(ws, name, args)
            self.log(f"[{i + 1}/{len(plan)}] MOVE {b} -> {tgt}")
            for _label, fw in self.pick_place_poses(ws, b, tgt):
                self.arm.move(fw)
            ws.apply_move(b, tgt, self.grid)
        self.arm.home()
        return ws

    # ---- manual GUI actions
    def pick(self, ws, block):
        up = np.array([0, 0, self.r["approach_height_m"]])
        p = ws.grasp_point(block)
        for xyz, g in [
            (p + up, self.r["grip_open_deg"]),
            (p, self.r["grip_open_deg"]),
            (p, self.r["grip_closed_deg"]),
            (p + up, self.r["grip_closed_deg"]),
        ]:
            self.arm.move(self._pose(xyz, g))

    def drop(self, ws, target):
        up = np.array([0, 0, self.r["approach_height_m"]])
        p = ws.place_point(target, self.grid)
        for xyz, g in [
            (p + up, self.r["grip_closed_deg"]),
            (p, self.r["grip_closed_deg"]),
            (p, self.r["grip_open_deg"]),
            (p + up, self.r["grip_open_deg"]),
        ]:
            self.arm.move(self._pose(xyz, g))
