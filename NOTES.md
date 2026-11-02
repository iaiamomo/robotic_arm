# Issues found while transcribing

The listings are kept verbatim. The issues below show up when the fragments are read together.
They could be errors that came in while the code was copied into the thesis, or bugs in the actual code.
They need checking against Agnese's real source.

## Blocking

1. **`get_pddl_base_data` — undefined names.** It assigns `sotto_id` / `sotto` but uses `under_id` / `under`
   (lines with `block_at` / `has_color ... MYSTERY` and `(on {obj_id} {under})`). This raises `NameError`.
   The variable names were probably translated only partly for the thesis.

2. **PDDL problem ↔ domain mismatch.**
   - The generated `:init` uses `(block_at X l1)`, which is not declared in `domain.pddl`.
     The domain models "on table" as `(on X cell)`.
   - `:objects` declares `l1 l2 - location` (and the example uses `target - location`), but the domain
     has type `cell`, not `location`.
   - The prompt tells the LLM to use `cell_n_n` objects, but no cell objects are emitted in `:objects`,
     and no `(clear cell_i_j)` facts are emitted in `:init`.
   As written, FF would reject the problem or find no plan for any goal that uses a cell.

3. **`parse_ff_output` drops every action with this domain.** The regex captures only the first token
   (`MOVE`), not `MOVE_...`. The `startswith("MOVE_")` filter then rejects it. The arguments
   (block, from, target) are never captured either. I tested this on a synthetic FF output and it returns `[]`.
   Either the real domain had actions with a different name (e.g. `move_to_cell`), or the parser differs from the listing.

## Minor

4. `problem_example.pddl` repeats `(has_color HIDDEN_UNDER_RED_5 MYSTERY)`, and nothing there marks the hidden block as on a cell.
5. `identify_cubes`: the GREEN branch has no `+10` margin, unlike BLUE and RED, so near-grey ROIs are biased toward GREEN.
   Listing 3.7 is captioned "frame differencing", but it does contour filtering and colour labelling.
6. `save_problem_file`: `"\n".join(...)` inside an f-string expression requires Python ≥ 3.12.
   The `try` block only catches I/O errors, not PDDL syntax errors, so the message "Synctatic error" is misleading.
7. `run_planner_wsl`: `shell=True` combined with a list argument works on Windows but is fragile. It is unnecessary here.
8. The thesis calls the prompt "few-shot", but the printed prompt contains no examples.
9. The firmware declares `wristRoll`, `wristPitch`, `elbow`, `shoulder` and `base` but never uses them. This is harmless.
   `startsWith("GRIP")` also accepts e.g. `GRIPPER 150`.
