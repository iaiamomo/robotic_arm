from types import SimpleNamespace

import numpy as np
import pytest

from braccio_ns.llm import LLMGoalTranslator
from braccio_ns.pddl import GoalError
from braccio_ns.sim import SimScene
from braccio_ns.system import System
from braccio_ns.world import Grid, build_world_state
from conftest import needs_ff


def scene(cfg, items):
    s, g = SimScene(cfg["vision"]["cube_size_m"]), Grid(cfg["grid"])
    for color, cell, level in items:
        s.add(color, *g.center(cell), level)
    return s


ITEMS = [
    ("RED", "CELL_0_0", 1),
    ("GREEN", "CELL_1_2", 1),
    ("BLUE", "CELL_0_4", 1),
    ("BLUE", "CELL_2_5", 1),
    ("RED", "CELL_2_5", 2),
]


def boot(cfg, goal="(:goal (and))", items=ITEMS):
    s = System(cfg, sim=True, goal=goal, log=lambda *_: None, scene=scene(cfg, items))
    s.capture_background()
    s.warmup(10)
    return s


def by_cell(s):
    g = Grid(s.cfg["grid"])
    mem = s.perception.target_memory()
    ws = build_world_state(mem, g, s.cfg["vision"]["cube_size_m"])
    return mem, ws, {ws.stack_cell(b): b for b in ws.blocks if ws.level(b) == 1}


def test_perception_detects_cubes_and_stack(cfg):
    s = boot(cfg)
    mem, ws, base = by_cell(s)
    assert sorted(v["color"] for v in mem.values()) == ["BLUE", "GREEN", "RED", "RED"]
    top = [k for k, v in mem.items() if v["level"] == 2]
    assert len(top) == 1 and mem[top[0]]["on_top_of"] == f"HIDDEN_UNDER_{top[0]}"
    assert {ws.stack_cell(b) for b in ws.blocks} == {"CELL_0_0", "CELL_1_2", "CELL_0_4", "CELL_2_5"}
    for k, v in mem.items():  # localisation error vs ground truth
        x, y = Grid(cfg["grid"]).center(ws.stack_cell(k))
        assert np.hypot(v["pos"][0] - x, v["pos"][1] - y) < 0.006


@needs_ff
def test_end_to_end_mission_moves_physical_cubes(cfg):
    s = boot(cfg)
    mem, ws, base = by_cell(s)
    red, green, blue = base["CELL_0_0"], base["CELL_1_2"], base["CELL_0_4"]
    top = next(k for k, v in mem.items() if v["level"] == 2)
    s.translator.goal = f"(:goal (and (on {green} {red}) (on {blue} HIDDEN_UNDER_{top})))"
    res = s.run_mission("ignored (fixed goal)")
    assert res.ok, res.error
    # the simulated camera must now SEE the new configuration, with the same IDs
    s.warmup(10)
    mem2 = s.perception.target_memory()
    assert mem2[green]["on_top_of"] == red and mem2[green]["level"] == 2
    assert mem2[blue]["on_top_of"] == f"HIDDEN_UNDER_{top}" and mem2[blue]["level"] == 2
    assert mem2[top]["level"] == 1
    # second mission on the updated world: tower of three on CELL_3_0
    s.translator.goal = f"(:goal (and (on {top} CELL_3_0) (on {red} {top}) (on {green} {red})))"
    res2 = s.run_mission("ignored")
    assert res2.ok, res2.error
    s.warmup(10)
    mem3 = s.perception.target_memory()
    assert mem3[green]["level"] == 3 and mem3[green]["on_top_of"] == red


@needs_ff
def test_unreachable_plan_is_rejected_before_moving(cfg):
    cfg["grid"]["origin_xy"] = [0.10, -0.0875]  # row 0 too close to the base
    s = boot(cfg, items=[("RED", "CELL_3_0", 1)])
    rid = next(iter(s.perception.target_memory()))
    s.translator.goal = f"(:goal (and (on {rid} CELL_0_0)))"
    res = s.run_mission("x")
    assert not res.ok and "unreachable" in res.error
    assert not any(c.startswith("MOVE") for c in s.arm.log)


class FakeLLM:
    def __init__(self, replies):
        self.replies, self.seen = list(replies), []
        self.chat = SimpleNamespace(completions=SimpleNamespace(create=self.create))

    def create(self, model, messages, temperature):
        self.seen.append(messages)
        assert temperature == 0.0
        msg = SimpleNamespace(content=self.replies.pop(0))
        return SimpleNamespace(choices=[SimpleNamespace(message=msg)])


def test_llm_translator_retries_on_invalid_goal(cfg):
    g = Grid(cfg["grid"])
    ws = build_world_state(
        {
            "RED_1": {"color": "RED", "pos": [0.3, 0, 0.03], "level": 1},
            "GREEN_2": {"color": "GREEN", "pos": [0.33, 0.05, 0.03], "level": 1},
        },
        g,
        0.03,
    )
    fake = FakeLLM(["(:goal (and (above GREEN_2 RED_1)))", "(:goal (and (on GREEN_2 RED_1)))"])
    tr = LLMGoalTranslator(cfg["llm"], cfg["planner"]["domain_file"], client=fake)
    goal, atoms, replies = tr.translate("green on red", ws, g, log=lambda *_: None)
    assert atoms == [("GREEN_2", "RED_1")] and len(replies) == 2
    system = fake.seen[0][0]["content"]
    assert "cardboard_block_world" in system and "CELL_3_5" in system

    fake = FakeLLM(["nope", "still nope"])
    tr = LLMGoalTranslator(cfg["llm"], cfg["planner"]["domain_file"], client=fake)
    with pytest.raises(GoalError):
        tr.translate("x", ws, g, log=lambda *_: None)
