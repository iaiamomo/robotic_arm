"""PDDL planner invocation and plan parsing (thesis §3.4.2).

Two backends, selected by `planner.backend`:
- "fast-downward" (default): Fast Downward from the `up-fast-downward` wheel, which ships
  a prebuilt binary for Windows/Linux/macOS, so no compiler or WSL is needed.
  Default search `astar(lmcut())` returns optimal (fewest-moves) plans.
- "ff": the thesis setup, an external Fast-Forward binary (`planner.command`, e.g. ["wsl", "./ff"]).

Fix w.r.t. Listing 3.13: actions are parsed with their arguments
(MOVE <block> <from> <target>); the listing kept only the action name.
"""

import importlib.util
import re
import subprocess
import sys
import tempfile
from pathlib import Path

from .config import REPO_ROOT, resolve

STEP_RE = re.compile(r"^(?:step\s+)?\d+\s*:\s*(.+)$", re.I)


class PlannerError(RuntimeError):
    pass


class GoalAlreadyTrue(Exception):
    pass


def parse_ff_output(stdout):
    """Return [(ACTION, [ARGS...]), ...]. Raises GoalAlreadyTrue / PlannerError."""
    low = stdout.lower()
    if "goal can be simplified to true" in low:
        raise GoalAlreadyTrue()
    if "goal can be simplified to false" in low or "problem proven unsolvable" in low:
        raise PlannerError("goal unreachable from the current state (FF: unsolvable)")
    if "found legal plan" not in low:
        raise PlannerError("FF produced no plan:\n" + stdout[-1500:])

    plan, collecting = [], False
    for line in stdout.splitlines():
        line = line.strip()
        if "found legal plan as follows" in line.lower():
            collecting = True
            continue
        if not collecting or not line:
            continue
        if line.lower().startswith("time spent"):
            break
        m = STEP_RE.match(line)
        if m:
            toks = m.group(1).strip("() ").split()
            plan.append((toks[0].upper(), [t.upper() for t in toks[1:]]))
    return plan


def _rel(path):
    """Path relative to the repo root with '/' separators, so the same arguments
    work for a native ff and for `wsl ./ff` launched from Windows."""
    p = resolve(path)
    try:
        return p.relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return p.as_posix()


def parse_sas_plan(text):
    """Fast Downward plan file -> [(ACTION, [ARGS...]), ...]. Lines look like
    `(move red_5 green_10 cell_0_0)`; the final `; cost = ...` line is a comment."""
    plan = []
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith(";"):
            continue
        toks = line.strip("() ").split()
        plan.append((toks[0].upper(), [t.upper() for t in toks[1:]]))
    return plan


# Fast Downward driver exit codes (driver/returncodes.py)
FD_UNSOLVABLE = {10, 11, 13}


def fast_downward_driver():
    """Path of fast-downward.py inside the installed up_fast_downward package (not imported)."""
    spec = importlib.util.find_spec("up_fast_downward")
    if spec is None or not spec.submodule_search_locations:
        raise PlannerError("Fast Downward is not installed: pip install up-fast-downward")
    driver = Path(spec.submodule_search_locations[0]) / "downward" / "fast-downward.py"
    if not driver.is_file():
        raise PlannerError(f"Fast Downward driver not found at {driver}")
    return driver


def run_fast_downward(planner_cfg, problem_file):
    with tempfile.TemporaryDirectory(prefix="braccio_fd_") as tmp:
        plan_file = Path(tmp) / "sas_plan"
        cmd = [
            sys.executable,
            str(fast_downward_driver()),
            "--plan-file",
            str(plan_file),
            str(resolve(planner_cfg["domain_file"])),
            str(resolve(problem_file)),
            "--search",
            planner_cfg.get("search", "astar(lmcut())"),
        ]
        try:
            res = subprocess.run(cmd, capture_output=True, text=True, cwd=tmp, timeout=planner_cfg.get("timeout_s", 60))
        except subprocess.TimeoutExpired as e:
            raise PlannerError("planner timeout") from e
        if res.returncode in FD_UNSOLVABLE:
            raise PlannerError("goal unreachable from the current state (Fast Downward: unsolvable)")
        if res.returncode != 0 or not plan_file.is_file():
            out = (res.stdout + res.stderr)[-1500:]
            raise PlannerError(f"Fast Downward produced no plan (exit code {res.returncode}):\n{out}")
        return parse_sas_plan(plan_file.read_text())


def run_planner(planner_cfg, problem_file=None):
    problem_file = problem_file or planner_cfg["problem_file"]
    backend = planner_cfg.get("backend", "fast-downward")
    if backend == "fast-downward":
        return run_fast_downward(planner_cfg, problem_file)
    if backend != "ff":
        raise PlannerError(f"unknown planner.backend {backend!r} (use 'fast-downward' or 'ff')")
    cmd = list(planner_cfg["command"]) + [
        "-o",
        _rel(planner_cfg["domain_file"]),
        "-f",
        _rel(problem_file),
    ]
    try:
        res = subprocess.run(
            cmd, capture_output=True, text=True, cwd=REPO_ROOT, timeout=planner_cfg.get("timeout_s", 60)
        )
    except FileNotFoundError as e:
        raise PlannerError(f"cannot run planner {cmd[0]!r}: {e}") from e
    except subprocess.TimeoutExpired as e:
        raise PlannerError("planner timeout") from e
    try:
        return parse_ff_output(res.stdout)
    except GoalAlreadyTrue:
        return []
