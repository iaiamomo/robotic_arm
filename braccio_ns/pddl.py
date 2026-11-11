"""PDDL problem generation consistent with pddl/domain.pddl.

Table positions are `cell` objects, "on table" is (on X CELL_r_c), and free
cells are declared (clear CELL_r_c) so blocks can be moved onto them.
"""

import re

DOMAIN_NAME = "cardboard_block_world"


def problem_init(ws, grid):
    objs = sorted(ws.blocks)
    colors = sorted(ws.colors())
    preds = []
    for bid in objs:
        b = ws.blocks[bid]
        preds.append(f"(has_color {bid} {b.color})")
        preds.append(f"(on {bid} {b.support})")
    for bid in objs:
        if ws.clear(bid):
            preds.append(f"(clear {bid})")
    occupied = ws.occupied_cells()
    for cell in grid.cells():
        if cell not in occupied:
            preds.append(f"(clear {cell})")
    return objs, colors, grid.cells(), preds


def render_problem(objs, colors, cells, init_preds, goal, name="tower"):
    return (
        f"(define (problem {name})\n"
        f"  (:domain {DOMAIN_NAME})\n"
        f"  (:objects\n"
        f"    {' '.join(objs)} - block\n"
        f"    {' '.join(colors)} - color\n"
        f"    {' '.join(cells)} - cell\n"
        f"  )\n"
        f"  (:init\n    " + "\n    ".join(init_preds) + "\n  )\n"
        f"  {goal}\n"
        f")\n"
    )


def save_problem_file(path, ws, grid, goal):
    objs, colors, cells, init = problem_init(ws, grid)
    text = render_problem(objs, colors, cells, init, goal)
    with open(path, "w") as f:
        f.write(text)
    return text


# ------------------------------------------------- goal parsing / validation
GOAL_ATOM = re.compile(r"\(\s*on\s+([A-Za-z0-9_]+)\s+([A-Za-z0-9_]+)\s*\)", re.I)


class GoalError(ValueError):
    pass


def extract_goal(text):
    """Pull a `(:goal ...)` s-expression out of an LLM reply (tolerates code fences / prose)."""
    i = text.lower().find("(:goal")
    if i < 0:
        raise GoalError("no '(:goal' found in the reply")
    depth = 0
    for j in range(i, len(text)):
        if text[j] == "(":
            depth += 1
        elif text[j] == ")":
            depth -= 1
            if depth == 0:
                return text[i : j + 1]
    raise GoalError("unbalanced parentheses in the goal")


def validate_goal(goal, blocks, cells):
    """Only (on X Y) atoms over known identifiers, X a block, X != Y. Returns
    (normalised_goal, atoms)."""
    body = goal.strip()[len("(:goal") : -1].strip()
    if body.lower().startswith("(and"):
        body = body[4:].rstrip()[:-1]
    leftovers = GOAL_ATOM.sub("", body)
    if re.search(r"\(", leftovers):
        bad = re.findall(r"\(\s*([A-Za-z_-]+)", leftovers)
        raise GoalError(f"only the 'on' predicate is allowed, found: {sorted(set(bad))}")
    atoms = [(a.upper(), b.upper()) for a, b in GOAL_ATOM.findall(body)]
    if not atoms:
        raise GoalError("empty goal")
    known = set(blocks) | set(cells)
    for a, b in atoms:
        if a not in blocks:
            raise GoalError(f"{a} is not a known block")
        if b not in known:
            raise GoalError(f"{b} is not a known block or cell")
        if a == b:
            raise GoalError(f"({a} on itself)")
    supports = [b for _, b in atoms]
    dup = {s for s in supports if supports.count(s) > 1}
    if dup:
        raise GoalError(f"more than one block placed on {sorted(dup)}")
    tops = [a for a, _ in atoms]
    if len(tops) != len(set(tops)):
        raise GoalError("a block appears with two different supports")
    norm = "(:goal (and " + " ".join(f"(on {a} {b})" for a, b in atoms) + "))"
    return norm, atoms
