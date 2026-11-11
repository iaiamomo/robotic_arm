"""start_mission: perception snapshot -> PDDL init -> LLM goal ->
problem file -> planner -> plan -> execution."""

from .config import resolve
from .pddl import save_problem_file
from .planner import run_planner
from .world import Grid, build_world_state


class MissionResult:
    def __init__(self, request, goal=None, plan=None, ok=False, error=None, problem_text=None):
        self.request, self.goal, self.plan, self.ok, self.error, self.problem_text = (
            request,
            goal,
            plan,
            ok,
            error,
            problem_text,
        )
        self.world = None

    def __repr__(self):
        return f"MissionResult(ok={self.ok}, goal={self.goal!r}, plan={self.plan}, error={self.error!r})"


class Mission:
    def __init__(self, cfg, translator, executor, log=print):
        self.cfg, self.translator, self.executor, self.log = cfg, translator, executor, log
        self.grid = Grid(cfg["grid"])

    def start_mission(self, request, target_memory, execute=True, stop_event=None):
        res = MissionResult(request)
        if not target_memory:
            res.error = "no objects in target memory"
            return res
        ws = build_world_state(target_memory, self.grid, self.cfg["vision"]["cube_size_m"])
        try:
            goal, atoms, _ = self.translator.translate(request, ws, self.grid, log=self.log)
            res.goal = goal
            self.log(f"Goal: {goal}")
            if ws.holds(atoms):
                self.log("Goal already satisfied, nothing to do.")
                res.plan, res.ok = [], True
                return res

            problem_path = resolve(self.cfg["planner"]["problem_file"])
            res.problem_text = save_problem_file(problem_path, ws, self.grid, goal)
            plan = run_planner(self.cfg["planner"], problem_path)
            res.plan = plan
            self.log("Plan:\n  " + "\n  ".join(f"{n} {' '.join(a)}" for n, a in plan))

            _, final = self.executor.dry_run(ws, plan)
            if not final.holds(atoms):
                raise RuntimeError("plan does not achieve the goal (symbolic check)")
            if execute:
                res.world = self.executor.execute(ws, plan, stop_event)
            res.ok = True
        except Exception as e:  # reported to GUI / CLI
            res.error = f"{type(e).__name__}: {e}"
            self.log(f"Mission failed: {res.error}")
        return res
