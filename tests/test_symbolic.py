import numpy as np
import pytest

from braccio_ns.pddl import GoalError, extract_goal, problem_init, save_problem_file, validate_goal
from braccio_ns.planner import GoalAlreadyTrue, PlannerError, parse_ff_output, run_planner
from braccio_ns.world import Grid, build_world_state
from conftest import needs_ff

MEM = {
    "RED_5": {"color": "red", "pos": [0.315, -0.0175, 0.06], "level": 2, "on_top_of": "HIDDEN_UNDER_RED_5"},
    "GREEN_10": {"color": "green", "pos": [0.285, 0.0525, 0.03], "level": 1, "on_top_of": None},
}


def world(cfg):
    g = Grid(cfg["grid"])
    return build_world_state(MEM, g, cfg["vision"]["cube_size_m"]), g


def test_world_state_hidden_block(cfg):
    ws, g = world(cfg)
    assert ws.blocks["HIDDEN_UNDER_RED_5"].color == "MYSTERY"
    assert ws.blocks["RED_5"].support == "HIDDEN_UNDER_RED_5"
    assert ws.blocks["HIDDEN_UNDER_RED_5"].support == "CELL_2_2"
    assert ws.blocks["GREEN_10"].support == "CELL_1_4"
    assert ws.level("RED_5") == 2 and not ws.clear("HIDDEN_UNDER_RED_5")


def test_problem_uses_only_domain_symbols(cfg):
    ws, g = world(cfg)
    objs, colors, cells, init = problem_init(ws, g)
    preds = {p.strip("()").split()[0] for p in init}
    assert preds <= {"has_color", "on", "clear"}
    assert "(clear CELL_2_2)" not in init and "(clear CELL_0_0)" in init
    assert "MYSTERY" in colors


def test_goal_validation(cfg):
    ws, g = world(cfg)
    blocks, cells = set(ws.blocks), set(g.cells())
    goal, atoms = validate_goal(extract_goal("Sure!\n```\n(:goal (and (on green_10 RED_5)))\n```"), blocks, cells)
    assert atoms == [("GREEN_10", "RED_5")] and goal == "(:goal (and (on GREEN_10 RED_5)))"
    assert validate_goal("(:goal (on RED_5 CELL_0_0))", blocks, cells)[1] == [("RED_5", "CELL_0_0")]
    for bad, msg in [("(:goal (and (on BLUE_1 RED_5)))", "not a known block"),
                     ("(:goal (and (above GREEN_10 RED_5)))", "only the 'on'"),
                     ("(:goal (and (on GREEN_10 CELL_9_9)))", "not a known block or cell"),
                     ("(:goal (and (on GREEN_10 RED_5) (on HIDDEN_UNDER_RED_5 RED_5)))", "more than one"),
                     ("no goal here", r"no '\(:goal'")]:
        with pytest.raises(GoalError, match=msg):
            validate_goal(extract_goal(bad), blocks, cells)


FF_OUT = """ff: parsing domain file
ff: found legal plan as follows

step    0: MOVE RED_5 HIDDEN_UNDER_RED_5 CELL_0_0
        1: MOVE GREEN_10 CELL_1_4 HIDDEN_UNDER_RED_5


time spent:    0.00 seconds instantiating 40 easy, 0 hard action templates
"""


def test_parse_ff_output_keeps_arguments():
    assert parse_ff_output(FF_OUT) == [("MOVE", ["RED_5", "HIDDEN_UNDER_RED_5", "CELL_0_0"]),
                                       ("MOVE", ["GREEN_10", "CELL_1_4", "HIDDEN_UNDER_RED_5"])]
    with pytest.raises(GoalAlreadyTrue):
        parse_ff_output("ff: goal can be simplified to TRUE. The empty plan solves it")
    with pytest.raises(PlannerError):
        parse_ff_output("ff: goal can be simplified to FALSE. No plan will solve it")


@needs_ff
def test_ff_solves_generated_problem(cfg, tmp_path):
    ws, g = world(cfg)
    goal = "(:goal (and (on GREEN_10 HIDDEN_UNDER_RED_5) (on RED_5 GREEN_10)))"
    save_problem_file(cfg["planner"]["problem_file"], ws, g, goal)
    plan = run_planner(cfg["planner"])
    assert len(plan) == 3
    for name, (b, frm, tgt) in plan:
        assert ws.blocks[b].support == frm
        ws.apply_move(b, tgt, g)
    assert ws.holds([("GREEN_10", "HIDDEN_UNDER_RED_5"), ("RED_5", "GREEN_10")])


@needs_ff
def test_thesis_example_problem_is_unsolvable(cfg):
    """Documents NOTES.md issue 2: Listing 3.9 is inconsistent with the domain."""
    from braccio_ns.config import REPO_ROOT
    with pytest.raises(PlannerError, match="unsolvable"):
        run_planner(cfg["planner"], REPO_ROOT / "thesis_listings" / "problem_example.pddl")
