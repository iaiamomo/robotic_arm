import os
from types import SimpleNamespace

import numpy as np
import pytest

from braccio_ns.llm import LLMGoalTranslator
from braccio_ns.pddl import GoalError
from braccio_ns.sim import SimScene
from braccio_ns.system import System
from braccio_ns.world import Grid, build_world_state
from conftest import needs_planner


def scene(cfg, items):
    s, g = SimScene(cfg["vision"]["cube_size_m"]), Grid(cfg["grid"])
    for color, cell, level in items:
        s.add(color, *g.center(cell), level)
    return s


ITEMS = [
    ("RED", "CELL_0_0", 1),
    ("GREEN", "CELL_1_2", 1),
    ("BLUE", "CELL_0_3", 1),
    ("BLUE", "CELL_2_4", 1),
    ("RED", "CELL_2_4", 2),
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
    assert {ws.stack_cell(b) for b in ws.blocks} == {"CELL_0_0", "CELL_1_2", "CELL_0_3", "CELL_2_4"}
    for k, v in mem.items():  # localisation error vs ground truth
        x, y = Grid(cfg["grid"]).center(ws.stack_cell(k))
        assert np.hypot(v["pos"][0] - x, v["pos"][1] - y) < 0.006


@needs_planner
def test_end_to_end_mission_moves_physical_cubes(cfg):
    s = boot(cfg)
    mem, ws, base = by_cell(s)
    red, green, blue = base["CELL_0_0"], base["CELL_1_2"], base["CELL_0_3"]
    top = next(k for k, v in mem.items() if v["level"] == 2)
    s.translator.goal = f"(:goal (and (on {green} {red}) (on {blue} HIDDEN_UNDER_{top})))"
    s.arm.log.clear()
    res = s.run_mission("ignored (fixed goal)")
    assert res.ok, res.error
    # starts with HOME + hand fully open; grasp and release are GRIP-only; after every grasp and
    # every release the arm goes back to the HOME joints with a MOVE that keeps the gripper as it is
    r = cfg["robot"]
    grip_open, grip_closed = f"GRIP {r['grip_open_deg']:.1f}", f"GRIP {r['grip_closed_deg']:.1f}"
    home_at = lambda g: "MOVE " + " ".join(f"{a:.1f}" for a in [g, *r["home"][1:]])  # noqa: E731
    log = s.arm.log
    assert log[:2] == ["HOME", grip_open] and log.count("HOME") == 1
    assert log.count(grip_closed) == len(res.plan)
    for i, c in enumerate(log):
        if c == grip_closed:  # grasp -> lift -> home holding
            assert log[i + 2] == home_at(r["grip_closed_deg"])
        if c == f"GRIP {r['grip_release_deg']:.1f}" and i > 1:  # release -> retract -> home, hand open
            assert log[i + 2] == home_at(r["grip_release_deg"])
    assert log[-1] == home_at(r["grip_release_deg"])
    # the simulated camera must now SEE the new configuration, with the same IDs
    s.warmup(10)
    mem2 = s.perception.target_memory()
    assert mem2[green]["on_top_of"] == red and mem2[green]["level"] == 2
    assert mem2[blue]["on_top_of"] == f"HIDDEN_UNDER_{top}" and mem2[blue]["level"] == 2
    # where `top` goes is the planner's choice (FF: table, Fast Downward: on green); perception must agree with it
    assert mem2[top]["on_top_of"] != f"HIDDEN_UNDER_{top}"
    assert mem2[top]["level"] == res.world.level(top)
    # second mission on the updated world: tower of three on CELL_2_0
    s.translator.goal = f"(:goal (and (on {top} CELL_2_0) (on {red} {top}) (on {green} {red})))"
    res2 = s.run_mission("ignored")
    assert res2.ok, res2.error
    s.warmup(10)
    mem3 = s.perception.target_memory()
    assert mem3[green]["level"] == 3 and mem3[green]["on_top_of"] == red


@needs_planner
def test_unreachable_plan_is_rejected_before_moving(cfg):
    cfg["grid"]["origin_xy"] = [0.10, -0.12]  # row 0 too close to the base
    s = boot(cfg, items=[("RED", "CELL_2_0", 1)])
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
    assert "cardboard_block_world" in system and "CELL_2_4" in system

    fake = FakeLLM(["nope", "still nope"])
    tr = LLMGoalTranslator(cfg["llm"], cfg["planner"]["domain_file"], client=fake)
    with pytest.raises(GoalError):
        tr.translate("x", ws, g, log=lambda *_: None)


def test_llm_api_key_from_env(monkeypatch):
    from braccio_ns.llm import api_key

    monkeypatch.setenv("MY_KEY", "sk-test")
    assert api_key({"api_key_env": "MY_KEY"}) == "sk-test"
    assert api_key({"api_key": "lm-studio", "api_key_env": "MY_KEY"}) == "lm-studio"
    monkeypatch.delenv("MY_KEY")
    with pytest.raises(RuntimeError, match="MY_KEY"):
        api_key({"api_key_env": "MY_KEY"})


def test_load_dotenv(tmp_path, monkeypatch):
    from braccio_ns.config import load_dotenv

    f = tmp_path / ".env"
    f.write_text('# comment\nA_KEY="sk-1"\nexport B_KEY=sk-2\nC_KEY=keep\n')
    monkeypatch.delenv("A_KEY", raising=False)
    monkeypatch.delenv("B_KEY", raising=False)
    monkeypatch.setenv("C_KEY", "env")
    assert load_dotenv(f) == ["A_KEY", "B_KEY"]
    assert os.environ["A_KEY"] == "sk-1" and os.environ["B_KEY"] == "sk-2" and os.environ["C_KEY"] == "env"


def test_manual_pick_grasps_the_cube(cfg):
    s = boot(cfg)
    mem, ws, base = by_cell(s)
    green = base["CELL_1_2"]
    s.arm.log.clear()
    s.executor.pick(ws, green)
    # HOME + hand fully open, above the cube, down (lower z), GRIP closed with the arm still, lift
    r = cfg["robot"]
    assert s.arm.log[:2] == ["HOME", f"GRIP {r['grip_open_deg']:.1f}"] and len(s.arm.log) == 6
    assert s.arm.log[4] == f"GRIP {r['grip_closed_deg']:.1f}" and s.arm.log[5].startswith("MOVE")
    z = [s.ik.fk_firmware([float(v) for v in m.split()[1:]])[2, 3] for m in s.arm.log[2:4]]
    assert z[1] < z[0]
    assert s.arm.held is not None and s.arm.held["color"] == "GREEN"
    s.arm.log.clear()
    s.executor.drop(ws, "CELL_2_0")
    assert s.arm.held is None
    # carried closed, released with a GRIP-only command at grip_release_deg, then retract
    grips = [float(m.split()[1]) for m in s.arm.log]
    assert grips == [cfg["robot"]["grip_closed_deg"]] * 2 + [cfg["robot"]["grip_release_deg"]] * 2
    assert s.arm.log[2] == f"GRIP {cfg['robot']['grip_release_deg']:.1f}"
    ws.apply_move(green, "CELL_2_0", s.grid)
    s.perception.sync(ws)  # as the GUI does after a drop: the cube keeps its ID
    s.warmup(10)
    mem2 = s.perception.target_memory()
    assert mem2[green]["level"] == 1
    assert build_world_state(mem2, s.grid, cfg["vision"]["cube_size_m"]).stack_cell(green) == "CELL_2_0"


def test_manual_pick_unreachable_does_not_move(cfg):
    from braccio_ns.executor import ExecutionError

    s = boot(cfg)
    mem, ws, base = by_cell(s)
    red = base["CELL_0_0"]
    ws.blocks[red].xy = np.array([0.05, 0.0])  # right next to the base: no IK solution
    s.arm.log.clear()
    with pytest.raises(ExecutionError, match=f"pick {red}"):
        s.executor.pick(ws, red)
    assert s.arm.log == []


def test_grid_projection_matches_camera(cfg):
    from braccio_ns.vision import project_to_image

    s = boot(cfg)
    tf, cam = s.perception.tf, s.camera
    g = Grid(cfg["grid"])
    pts = np.array([[*g.center(c), 0.0] for c in g.cells()])
    assert np.allclose(tf(tf.to_camera(pts)), pts)
    uv, z = project_to_image(cam.intr, tf.to_camera(pts))
    assert (z > 0).all() and np.allclose(uv, [cam.project(p)[0] for p in pts], atol=1e-6)
    assert s.perception.overlay(show_grid=True).shape == s.perception.overlay().shape


def test_calibrate_applies_new_transform(cfg):
    s = boot(cfg)
    old = s.perception.tf
    tf = s.calibrate()
    assert s.perception.tf is tf and tf is not old and tf.rmse < 0.005
    s.warmup(10)  # tracks were reset and are re-acquired with the new transform
    assert sorted(v["color"] for v in s.perception.target_memory().values()) == ["BLUE", "GREEN", "RED", "RED"]
