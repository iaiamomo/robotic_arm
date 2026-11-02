# Braccio++ Neuro-Symbolic Manipulation

Natural-language control of an Arduino Braccio++ arm through an
LLM → PDDL → Fast-Forward → serial pipeline, with Intel RealSense D435i perception.
Based on the Master's thesis of **Agnese Maria Capparelli** (Sapienza Università di Roma).

> **Provenance — read this first.**
> - `pddl/domain.pddl` is Agnese's domain, unchanged. `firmware/` is her Listing 3.1 with input validation added.
> - `braccio_ns/` implements the system described in the thesis. The thesis prints only fragments of the
>   Python code, so this package is a re-implementation, not Agnese's original source.
>   [`NOTES.md`](NOTES.md) lists the issues found in the printed listings and every design choice.
> - It has been tested end-to-end **in simulation only**: synthetic RGB-D camera, simulated arm,
>   and the real FF planner. All values marked `CALIBRATE` in `config/default.json` must be
>   measured on the real setup.

## Pipeline

```
RealSense ─► vision (bg subtraction, colour, top-face 3D) ─► tracker ─► target_memory ─► PDDL :init
User NL command ─► LLM (Llama 3.2 @ LM Studio, T=0, validated) ───────────────────────► PDDL :goal
domain.pddl + problem.pddl ─► FF ─► plan ─► reachability check ─► IK ─► serial ─► Braccio++ firmware
                                                     └─► tracker re-synced with the new state
```

## Layout

| Path | Origin | Content |
|---|---|---|
| `firmware/braccio_serial/` | thesis (Listing 3.1) + validation | Serial command firmware: `HOME`, `GRIP g`, `MOVE g r p e s b` |
| `pddl/domain.pddl` | thesis (Listing 3.8) | Blocks-world domain |
| `braccio_ns/serial_link.py` | reconstructed | pyserial client for the firmware protocol |
| `braccio_ns/kinematics.py` | reconstructed | IK (analytic, verified with IKPy FK on the URDF) + firmware angle mapping |
| `urdf/braccio_plusplus.urdf` | reconstructed | `braccio.urdf` was not in the thesis — **approximate** link lengths |
| `braccio_ns/calibration.py` | reconstructed | Eye-to-hand calibration: ArUco, 33 points × 20 frames, Umeyama |
| `braccio_ns/camera.py` | reconstructed | RealSense wrapper (depth aligned to colour) |
| `braccio_ns/vision.py` | thesis + extended | Background subtraction, colour classification, 3D top-face localisation, tracker |
| `braccio_ns/world.py` | reconstructed | Symbolic world model, grid cells, hidden blocks |
| `braccio_ns/pddl.py` | thesis, fixed | Problem generation consistent with the domain; goal validation |
| `braccio_ns/planner.py` | thesis, fixed | FF invocation (native or `wsl ./ff`) and plan parsing with arguments |
| `braccio_ns/llm.py` | thesis prompt + reconstructed | LM Studio client, few-shot examples, validation + 1 retry |
| `braccio_ns/executor.py` | reconstructed | MOVE → pick-and-place trajectory; whole-plan IK check before moving |
| `braccio_ns/mission.py` | reconstructed | `start_mission` |
| `braccio_ns/gui.py` | reconstructed | Tkinter GUI as described in §3.1 |
| `braccio_ns/sim.py` | new | Synthetic RGB-D camera + simulated arm (FK-driven grasp/release) |
| `tests/` | new | Python tests (symbolic, geometry, vision, end-to-end missions) + host-compiled firmware parser tests |

## Setup

```bash
pip install -e ".[realsense]"        # or: pip install -r requirements.txt
# FF planner (thesis setup: inside WSL on Windows)
#   build FF-v2.3; on modern gcc add -fcommon to CFLAGS in the makefile
#   put the `ff` binary in the repo root  ->  planner.command = ["wsl", "./ff"]  (default)
#   or on Linux/macOS:                       --ff /path/to/ff
```
Flash `firmware/braccio_serial/braccio_serial.ino` with the Arduino IDE (library `Arduino_Braccio_plusplus`).
Start LM Studio's local server with the Llama 3.2 Instruct model, and set `llm.model` to the identifier LM Studio shows.

## Usage

```bash
python -m braccio_ns marker                       # print ArUco id 0 and attach it to the end-effector
python -m braccio_ns --port COM3 calibrate        # writes config/calibration.json (RMSE printed)
python -m braccio_ns --port COM3 gui              # full system
python -m braccio_ns --port COM3 mission "build a tower with the red block on top" --dry-run
python -m braccio_ns reach                        # which grid cells the arm can reach, and with which tilt

# without hardware
python -m braccio_ns --ff ./ff gui --sim
python -m braccio_ns --ff ./ff mission "..." --sim --goal "(:goal (and (on GREEN_2 RED_4)))"
FF_PATH=./ff pytest -q
```
`--goal` bypasses the LLM, which is useful to test perception, planning and the arm on their own.
After `pip install -e .`, the `braccio-ns` command can be used instead of `python -m braccio_ns`.

## Development

```bash
pip install -e ".[dev]"
ruff check braccio_ns tests && ruff format --check braccio_ns tests
FF_PATH=/path/to/ff pytest -q       # tests that need FF are skipped without it
tests/firmware/run_tests.sh         # firmware command parser, compiled with g++ against mocks
```
GitHub Actions (`.github/workflows/ci.yml`) runs the following on every push:
- builds FF;
- runs lint and the tests on Python 3.10 and 3.12;
- runs the firmware parser tests;
- compiles the sketch for the Braccio++ board (`arduino:mbed_nano:nanorp2040connect`).

## Before running on the real arm (`CALIBRATE`)

1. **URDF link lengths.** Measure base→shoulder, shoulder→elbow, elbow→wrist, and wrist→grasp point.
2. **Angle mapping.** `robot.joint_offsets_deg` and `joint_signs` give firmware_deg = offset + sign·deg(q).
   The defaults assume 157.5° = link straight up and the positive direction = leaning away from the base.
   Check this joint by joint with `MOVE` commands from the serial monitor.
3. **Gripper.** Set `grip_open_deg` and `grip_closed_deg` for the cardboard cubes.
4. **Grid.** `grid.origin_xy`, `row_step_xy` and `col_step_xy` give the centre of `CELL_0_0` and the steps,
   in the robot frame. Then run `reach`.
5. **Cube size and vision thresholds.** `vision.cube_size_m`, `min_area_px`, `aspect_*`.
6. **Marker offset.** `calibration.marker_offset_tip_m` is where the ArUco centre sits relative to the grasp point (tip frame).

## Known limitations

- With the placeholder geometry and the firmware joint limits, the arm cannot grasp vertically on the table.
  IK picks the smallest feasible tilt (25–60°). Targets closer than ~0.25 m to the base are unreachable.
- A camera in front of the arm cannot see cubes behind a stack. Those cubes are missing from `:init`.
- There is no closed-loop replanning (thesis §5, future work). The executor works open-loop.
  However, the whole plan is checked for reachability before the arm moves, and the tracker is
  re-synchronised afterwards, so object IDs persist across missions.
