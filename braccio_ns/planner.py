"""Fast-Forward planner invocation and plan parsing (thesis §3.4.2).

Fix w.r.t. Listing 3.13: actions are parsed with their arguments
(MOVE <block> <from> <target>); the listing kept only the action name.
"""
import re
import subprocess

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


def run_planner(planner_cfg, problem_file=None):
    cmd = list(planner_cfg["command"]) + [
        "-o", _rel(planner_cfg["domain_file"]),
        "-f", _rel(problem_file or planner_cfg["problem_file"]),
    ]
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, cwd=REPO_ROOT,
                             timeout=planner_cfg.get("timeout_s", 60))
    except FileNotFoundError as e:
        raise PlannerError(f"cannot run planner {cmd[0]!r}: {e}")
    except subprocess.TimeoutExpired:
        raise PlannerError("planner timeout")
    try:
        return parse_ff_output(res.stdout)
    except GoalAlreadyTrue:
        return []
