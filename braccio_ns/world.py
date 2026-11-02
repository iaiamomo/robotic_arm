"""Symbolic world model shared by perception, PDDL generation and execution.

Blocks are identified as <COLOR>_<n> (e.g. RED_5). A block seen at level >= 2
with no visible block underneath gets a hidden support HIDDEN_UNDER_<ID> of
colour MYSTERY (thesis Listing 3.9/3.10 convention). Table positions are the
grid cells CELL_<row>_<col> (the cell_n_n identifiers in the LLM prompt).
"""
from dataclasses import dataclass, field

import numpy as np


class Grid:
    def __init__(self, grid_cfg):
        self.rows, self.cols = grid_cfg["rows"], grid_cfg["cols"]
        self.origin = np.asarray(grid_cfg["origin_xy"], float)
        self.row_step = np.asarray(grid_cfg["row_step_xy"], float)
        self.col_step = np.asarray(grid_cfg["col_step_xy"], float)

    @staticmethod
    def name(r, c):
        return f"CELL_{r}_{c}"

    def cells(self):
        return [self.name(r, c) for r in range(self.rows) for c in range(self.cols)]

    def center(self, cell):
        _, r, c = cell.split("_")
        return self.origin + int(r) * self.row_step + int(c) * self.col_step

    def nearest(self, xy):
        xy = np.asarray(xy[:2], float)
        best = min(self.cells(), key=lambda n: np.linalg.norm(self.center(n) - xy))
        return best, float(np.linalg.norm(self.center(best) - xy))


@dataclass
class Block:
    id: str
    color: str
    support: str              # block id or cell name
    xy: np.ndarray
    hidden: bool = False


@dataclass
class WorldState:
    blocks: dict = field(default_factory=dict)   # id -> Block
    cube: float = 0.03

    # ---- queries
    def stack_cell(self, bid):
        s = self.blocks[bid].support
        return s if s.startswith("CELL_") else self.stack_cell(s)

    def level(self, bid):
        s = self.blocks[bid].support
        return 1 if s.startswith("CELL_") else 1 + self.level(s)

    def clear(self, obj):
        return not any(b.support == obj for b in self.blocks.values())

    def occupied_cells(self):
        return {b.support for b in self.blocks.values() if b.support.startswith("CELL_")}

    def top_z(self, obj):
        """Height of the top surface of a block (or of the table for a cell)."""
        return 0.0 if obj.startswith("CELL_") else self.level(obj) * self.cube

    def grasp_point(self, bid):
        b = self.blocks[bid]
        return np.array([b.xy[0], b.xy[1], self.top_z(bid) - self.cube / 2])

    def place_point(self, target, grid):
        xy = grid.center(target) if target.startswith("CELL_") else self.blocks[target].xy
        return np.array([xy[0], xy[1], self.top_z(target) + self.cube / 2])

    # ---- update (symbolic effect of MOVE)
    def apply_move(self, bid, target, grid):
        b = self.blocks[bid]
        b.xy = (grid.center(target) if target.startswith("CELL_") else self.blocks[target].xy).copy()
        b.support = target

    def holds(self, goal_atoms):
        return all(self.blocks.get(a) is not None and self.blocks[a].support == b for a, b in goal_atoms)

    def colors(self):
        return {b.color for b in self.blocks.values()}


def build_world_state(memory, grid, cube):
    """target_memory -> WorldState.

    memory: {id: {'color', 'pos': [x, y, z_top], 'level', 'on_top_of'}} (robot frame).
    """
    ws = WorldState(cube=cube)
    for oid, d in memory.items():
        ws.blocks[oid] = Block(oid, d["color"].upper(), support="", xy=np.asarray(d["pos"][:2], float))

    for oid, d in memory.items():
        under = d.get("on_top_of")
        level = int(d.get("level", 1))
        if under and under in memory:
            ws.blocks[oid].support = under
            continue
        # Fill the stack below with hidden blocks (level-1 of them), bottom one on a cell
        prev = oid
        for k in range(1, level):
            hid = f"HIDDEN_UNDER_{oid}" if k == 1 else f"HIDDEN_UNDER_{oid}_{k}"
            ws.blocks[hid] = Block(hid, "MYSTERY", support="", xy=ws.blocks[oid].xy.copy(), hidden=True)
            ws.blocks[prev].support = hid
            prev = hid
        ws.blocks[prev].support = grid.nearest(d["pos"])[0]
    return ws
