# Notes on the thesis code and on the reconstruction

## A. Issues in the thesis listings (`thesis_listings/`)

These could be errors made when the code was copied into the thesis, or bugs in the actual code.
They need checking against Agnese's real source.

1. **`get_pddl_base_data` — undefined names.** It assigns `sotto_id` / `sotto` but uses `under_id` / `under`. This raises `NameError`.
2. **Problem ↔ domain mismatch.** The listing uses `block_at` (not declared in the domain) and type `location`
   (the domain has `cell`). It emits no cell objects and no `(clear cell)` facts.
   *Verified:* FF on `thesis_listings/problem_example.pddl` returns "goal can be simplified to FALSE"
   (see `tests/test_symbolic.py::test_thesis_example_problem_is_unsolvable`).
3. **`parse_ff_output` returns an empty plan with this domain.** The regex captures only `MOVE`, which then fails the `startswith("MOVE_")` filter, and the arguments are lost.
4. `problem_example.pddl` repeats `(has_color HIDDEN_UNDER_RED_5 MYSTERY)`.
5. `identify_cubes`: GREEN has no margin, unlike RED and BLUE.
6. `save_problem_file`: `"\n"` inside an f-string expression requires Python ≥ 3.12.
7. `run_planner_wsl`: `shell=True` combined with a list argument.
8. The prompt is described as few-shot, but no examples are printed.
9. Firmware: the motor objects other than `gripper` are unused, and `startsWith("GRIP")` also accepts `GRIPPER x`.

## B. Design choices in `braccio_ns/` (where the thesis was silent or inconsistent)

| Topic | Choice | Why |
|---|---|---|
| Table positions | `CELL_r_c` objects of type `cell`; on table = `(on X CELL_r_c)` | Matches the thesis domain unchanged, and the prompt's `cell_n_n` |
| Hidden blocks | `HIDDEN_UNDER_<ID>`, colour `MYSTERY`, one per missing level | Thesis convention (Listings 3.9/3.10) |
| IK | Closed-form planar solution, verified with IKPy FK on the URDF | IKPy's numerical solver got stuck in local minima (inconsistent results on adjacent cells) |
| Grasp orientation | Smallest feasible tilt from vertical | A vertical grasp is infeasible within the firmware joint limits for the assumed link lengths |
| Foreground mask | Max per-channel difference (`diff_mode: "max"`); the thesis grayscale mode is available | In grayscale, blue's low luminance lets its faces fall below threshold 50 |
| Colour | Per-pixel chromatic dominance, then components per colour | With a 45° camera a stack forms one two-coloured blob; the thesis rule used the blob's mean colour |
| 3D position | Deproject all blob pixels and take the mean of the top-face points | The 2D centroid of top + front face sits near the front edge, an error of about half a cube toward the camera |
| Covered cubes | A component under another (xy overlap, lower top) is dropped | Its side face would otherwise create a spurious block; the cube becomes `HIDDEN_UNDER_*` |
| Tracking | Per-colour nearest neighbour, min hits 3, max misses 15; occluded tracks do not age; tracks are re-synced after execution | Addresses ID flickering (thesis §5) |
| LLM output | `(:goal ...)` extracted, then validated: only `on`, only known IDs, at most one block per support; 1 corrective retry | Makes "translation failed" explicit instead of failing in FF |
| Execution | Whole plan simulated and IK-checked before the first movement | The thesis reports failures from unreachable placements (§4.3) |
| FF on Windows | Runs from the repo root with relative POSIX paths, so `wsl ./ff` works | As in the thesis setup |

## C. What the simulation does and does not validate

Validated: the data flow and consistency of all modules, PDDL generated from perception, FF plans,
IK/FK round-trips within firmware limits, and the physical outcome of the plan as re-observed by the synthetic camera.

Not validated:
- real optics, lighting and depth noise;
- link lengths, angle offsets and directions, and gripper values of the real Braccio++;
- the timing of `move_settle_s`;
- the real LM Studio model's output quality (only a mocked client is tested).
