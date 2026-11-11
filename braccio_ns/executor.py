"""Plan execution: symbolic MOVE actions -> pick-and-place trajectories -> serial MOVE commands.

Every action of the plan is checked for IK feasibility on a copy of the world
*before* the arm moves, so an unreachable placement aborts the mission instead
of failing half-way.
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

    def _grasp_point(self, ws, block):
        """The fingers close grasp_below_center_m below the cube centre, around more of the cube."""
        return ws.grasp_point(block) - [0, 0, self.r.get("grasp_below_center_m", 0.0)]

    def _place_point(self, ws, target):
        """Lowered by the same amount, so the held cube still lands at the right height."""
        return ws.place_point(target, self.grid) - [0, 0, self.r.get("grasp_below_center_m", 0.0)]

    def _release_deg(self):
        """Release opens wider than the approach, so a firmly squeezed cube is let go."""
        return self.r.get("grip_release_deg", self.r["grip_open_deg"])

    def _home_fw(self, grip_deg):
        """HOME joints with the gripper at grip_deg (firmware order)."""
        return [grip_deg, *self.r["home"][1:]]

    def pick_place_poses(self, ws, block, target):
        """One MOVE action: pick, back to HOME holding the block (grip unchanged), place,
        back to HOME with the hand as the release left it. Passing through HOME keeps the
        transfers clear of the other blocks."""
        up = np.array([0, 0, self.r["approach_height_m"]])
        g_open, g_closed, g_rel = self.r["grip_open_deg"], self.r["grip_closed_deg"], self._release_deg()
        pick = self._grasp_point(ws, block)
        place = self._place_point(ws, target)
        return [
            ("above pick", self._pose(pick + up, g_open)),
            ("descend", self._pose(pick, g_open)),
            ("grasp", self._pose(pick, g_closed)),
            ("lift", self._pose(pick + up, g_closed)),
            ("home holding", self._home_fw(g_closed)),
            ("above place", self._pose(place + up, g_closed)),
            ("lower", self._pose(place, g_closed)),
            ("release", self._pose(place, g_rel)),
            ("retract", self._pose(place + up, g_rel)),
            ("home", self._home_fw(g_rel)),
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

    def home_open(self):
        """HOME with an empty hand, then open the gripper fully in place: the firmware HOME
        sets the gripper to its home angle (half closed), which must not stay after a drop."""
        self.arm.home()
        self.arm.grip(self.r["grip_open_deg"])

    def prepare_pick(self):
        """Before every pick: back to HOME (clear of the blocks, so the transfer cannot sweep
        them) and open the gripper fully."""
        self.home_open()

    def home_holding(self):
        """HOME pose with the gripper kept closed (the firmware HOME would loosen it)."""
        self.arm.move(self._home_fw(self.r["grip_closed_deg"]))

    def _send(self, poses):
        """Send firmware poses in order. When only the gripper changes (same joints as the
        previous pose: grasp / release), use GRIP so the hand moves alone with the arm still."""
        prev = None
        for fw in poses:
            if prev is not None and np.allclose(fw[1:], prev[1:]) and fw[0] != prev[0]:
                self.arm.grip(fw[0])
            else:
                self.arm.move(fw)
            prev = fw

    # ---- execution
    def execute(self, ws, plan, stop_event=None):
        """Start from HOME with the hand fully open; every pick and every place ends back at
        HOME without touching the gripper (see pick_place_poses)."""
        self.dry_run(ws, plan)
        self.prepare_pick()
        g_open, g_rel = self.r["grip_open_deg"], self._release_deg()
        for i, (name, args) in enumerate(plan):
            if stop_event is not None and stop_event.is_set():
                self.log("Mission stopped by user")
                break
            b, _, tgt = self._check_action(ws, name, args)
            self.log(f"[{i + 1}/{len(plan)}] MOVE {b} -> {tgt}")
            if i > 0 and g_rel != g_open:
                self.arm.grip(g_open)  # at HOME, arm still: approach opening if it differs from the release one
            self._send([fw for _label, fw in self.pick_place_poses(ws, b, tgt)])
            ws.apply_move(b, tgt, self.grid)
        return ws

    # ---- manual GUI actions
    def _move_through(self, what, waypoints, before=None):
        """IK for every waypoint first, so an unreachable one aborts before the arm moves."""
        try:
            poses = [self._pose(xyz, g) for xyz, g in waypoints]
        except Unreachable as e:
            raise ExecutionError(f"{what}: {e}") from e
        if before is not None:
            before()
        self._send(poses)

    def pick(self, ws, block):
        """HOME + GRIP fully open, above the block, descend, GRIP closed, lift."""
        up = np.array([0, 0, self.r["approach_height_m"]])
        p = self._grasp_point(ws, block)
        g_open, g_closed = self.r["grip_open_deg"], self.r["grip_closed_deg"]
        waypoints = [(p + up, g_open), (p, g_open), (p, g_closed), (p + up, g_closed)]
        self._move_through(f"pick {block}", waypoints, before=self.prepare_pick)

    def drop(self, ws, target):
        """Above the target, lower, GRIP fully open (release), retract."""
        up = np.array([0, 0, self.r["approach_height_m"]])
        p = self._place_point(ws, target)
        g_closed, g_rel = self.r["grip_closed_deg"], self._release_deg()
        self._move_through(f"drop on {target}", [(p + up, g_closed), (p, g_closed), (p, g_rel), (p + up, g_rel)])
