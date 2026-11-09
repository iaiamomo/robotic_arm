# Braccio++ Neuro-Symbolic Manipulation

Natural-language control of an Arduino Braccio++ arm through an
LLM → PDDL → Fast Downward → serial pipeline, with Intel RealSense D435i perception.
Based on the Master's thesis of **Agnese Maria Capparelli** (Sapienza Università di Roma).

> Tested end-to-end **in simulation**: synthetic RGB-D camera, simulated arm, and a real PDDL planner
> (Fast Downward). Values marked `CALIBRATE` in `config/default.json` must be measured on the real setup
> (see [Before running on the real arm](#before-running-on-the-real-arm-calibrate)).

## Requirements

- **Python 3.10 – 3.14. Python 3.14 is the maximum: do not use 3.15 or newer.** `pyproject.toml` enforces
  this (`requires-python = ">=3.10,<3.15"`), so `pip install` refuses newer interpreters.
- Arduino Braccio++ flashed with `firmware/braccio_serial/braccio_serial.ino`
  (Arduino IDE, library `Arduino_Braccio_plusplus`).
- Intel RealSense D435i.
- An [OpenRouter](https://openrouter.ai) API key for the LLM (or a local LM Studio server).

## Pipeline

```
RealSense ─► vision (bg subtraction, colour, top-face 3D) ─► tracker ─► target_memory ─► PDDL :init
User NL command ─► LLM (Llama 3.2 @ OpenRouter, T=0, validated) ───────────────────────► PDDL :goal
domain.pddl + problem.pddl ─► Fast Downward ─► plan ─► reachability check ─► IK ─► serial ─► Braccio++ firmware
                                                     └─► tracker re-synced with the new state
```

## Layout

| Path | Content |
|---|---|
| `firmware/braccio_serial/` | Serial command firmware: `HOME`, `GRIP g`, `MOVE g r p e s b` |
| `pddl/domain.pddl` | Blocks-world domain |
| `braccio_ns/serial_link.py` | pyserial client for the firmware protocol |
| `braccio_ns/kinematics.py` | IK (analytic, verified with IKPy FK on the URDF) + firmware angle mapping |
| `urdf/braccio_plusplus.urdf` | Kinematic model — **approximate** link lengths, to be measured |
| `braccio_ns/calibration.py` | Eye-to-hand calibration: ArUco on the gripper, 33 poses × 20 frames, Umeyama |
| `braccio_ns/camera.py` | RealSense wrapper (depth aligned to colour) |
| `braccio_ns/vision.py` | Background subtraction, colour classification, 3D top-face localisation, tracker, preview overlay with the grid |
| `braccio_ns/world.py` | Symbolic world model, grid cells, hidden blocks |
| `braccio_ns/pddl.py` | Problem generation consistent with the domain; goal validation |
| `braccio_ns/planner.py` | Fast Downward (default) or FF (`wsl ./ff`) invocation, plan parsing with arguments |
| `braccio_ns/llm.py` | OpenAI-compatible client (OpenRouter), prompt with few-shot examples, validation + 1 retry |
| `braccio_ns/executor.py` | MOVE → pick-and-place trajectory through HOME; whole-plan IK check before moving |
| `braccio_ns/mission.py` | `start_mission` |
| `braccio_ns/gui.py` | Tkinter GUI with embedded camera view, manual pick/drop, calibration |
| `braccio_ns/sim.py` | Synthetic RGB-D camera + simulated arm (FK-driven grasp/release) |
| `tests/` | Python tests (symbolic, geometry, vision, end-to-end missions) + host-compiled firmware parser tests |

## Setup

Create the virtual environment with Python 3.14 (or any version from 3.10 to 3.14):

```bash
py -3.14 -m venv .venv                     # Windows   (Linux/macOS: python3.14 -m venv .venv)
.venv\Scripts\activate                     # Windows   (Linux/macOS: source .venv/bin/activate)
pip install -e ".[realsense]"              # or: pip install -r requirements.txt
```

**Planner.** Fast Downward is installed with the `up-fast-downward` dependency (prebuilt, also on Windows):
nothing else to do. Optional: build FF-v2.3 (on modern gcc add `-fcommon` to CFLAGS in the
makefile), then use `--ff /path/to/ff`, or set `planner.backend = "ff"` and `planner.command = ["wsl", "./ff"]`.

**LLM.** OpenRouter with `meta-llama/llama-3.2-3b-instruct` (`llm.model`). Put the key in a `.env` file in
the repo root (git-ignored, loaded at startup; see `.env.example`), or set it in the shell:
```
OPENROUTER_API_KEY=sk-or-...               # .env
$env:OPENROUTER_API_KEY = "sk-or-..."      # PowerShell (setx OPENROUTER_API_KEY "sk-or-..." to make it permanent)
export OPENROUTER_API_KEY=sk-or-...        # bash
```
To use a local LM Studio server instead, set `llm.base_url` to `http://localhost:1234/v1`, `llm.api_key`
to `"lm-studio"` and `llm.model` to the identifier LM Studio shows.

## Usage

```bash
python -m braccio_ns --port COM3 gui              # full system
python -m braccio_ns --port COM3 mission "build a tower with the red block on top" --dry-run
python -m braccio_ns --port COM3 calibrate        # writes config/calibration.json (RMSE printed)
python -m braccio_ns --port COM3 grip 200 150 200 # gripper test: open, close, open
python -m braccio_ns marker                       # print ArUco id 0 to attach to the gripper
python -m braccio_ns reach                        # which grid cells the arm can reach, and with which tilt
python -m braccio_ns ik CELL_1_2 --above          # MOVE command for a cell (or X Y Z in metres); see test_movimenti.md

# without hardware
python -m braccio_ns gui --sim
python -m braccio_ns mission "..." --sim --goal "(:goal (and (on GREEN_2 RED_4)))"
pytest -q
```
`--goal` bypasses the LLM, which is useful to test perception, planning and the arm on their own.
After `pip install -e .`, the `braccio-ns` command can be used instead of `python -m braccio_ns`.
Only one program at a time can open the serial port: close the Arduino serial monitor first.

### GUI

- **Camera** (right): live view with the detected cubes and their IDs; *Show grid* draws the grid cells
  projected on the table (needs a calibration). Before the first *Update Background* the view shows a reminder.
- **Detected objects** (left): **double-click** an object (or select it and press Enter) to pick it:
  HOME, hand fully open, above the cube, down, close, lift.
- **Drop on target**: choose a cell or a block, then the arm lowers it there and opens the hand fully.
- **Command + Start Mission**: natural-language mission through LLM → planner → arm.
- **Update Background**: capture the empty workspace (do this first, with no cubes on the table).
- **Home**: back to HOME with the hand open, or keeping the cube if one is held.
- **Calibrate**: eye-to-hand calibration (asks for confirmation); the result is saved and applied at once.

### Arm motion

Missions start with `HOME` + hand fully open. For every action of the plan the arm goes above the block,
descends, closes the hand (`GRIP`, arm still), lifts, **goes back to HOME keeping the grip**, moves above
the target, lowers, opens the hand fully (`GRIP`), lifts and **goes back to HOME with the hand open**.
Returning to HOME uses a `MOVE` to the home joints, because the firmware `HOME` command also moves the gripper.

## Development

```bash
pip install -e ".[dev]"
ruff check braccio_ns tests && ruff format --check braccio_ns tests
pytest -q                           # FF_PATH=/path/to/ff runs the planner tests with FF instead
tests/firmware/run_tests.sh         # firmware command parser, compiled with g++ against mocks
```
GitHub Actions (`.github/workflows/ci.yml`) runs the following on every push:
- runs lint and the tests on Python 3.10 and 3.12;
- runs the firmware parser tests;
- compiles the sketch for the Braccio++ board (`arduino:mbed_nano:nanorp2040connect`).

## Before running on the real arm (`CALIBRATE`)

1. **URDF link lengths.** Measure base→shoulder, shoulder→elbow, elbow→wrist, and wrist→grasp point.
   They decide how far the arm reaches (see `reach`).
2. **Angle mapping.** `robot.joint_offsets_deg` and `joint_signs` give firmware_deg = offset + sign·deg(q).
   The base offset is 95°, the base angle of HOME, so HOME points along the grid's centre column.
   The other joints assume 157.5° = link straight up. Check joint by joint with `MOVE` commands.
3. **Gripper.** Measured: `grip_open_deg` = `grip_release_deg` = 200 (fully open), `grip_closed_deg` = 150.
   Higher = more open. Do not use the firmware limits 120/230: the gripper servo stalls and stops responding
   until the board is power-cycled. Test with `python -m braccio_ns --port COM3 grip ...`.
4. **Grid.** 3 rows × 5 columns, cells 5 cm (rows) × 6 cm (columns). `grid.origin_xy` = centre of `CELL_0_0`
   in the robot frame: row 0 is 26 cm from the base, column 2 (`CELL_r_2`) is straight in front of HOME.
   Mark it on the table, then check that the grid drawn in the GUI camera view matches it.
5. **Cube size and vision thresholds.** `vision.cube_size_m`, `min_area_px`, `aspect_*`.
6. **Calibration marker.** Print it with `marker` and attach it to the gripper, facing the camera.
   `calibration.marker_offset_tip_m` is its centre in the tip frame: measured 10 cm back from the grasp
   point along the gripper (`[0, 0, -0.1]`). If it sits off the gripper axis, add that lateral offset too.
   The calibration poses keep the grasp point 5–13 cm above the table (`calibration.tip_heights_m`).

## Known limitations

- With the placeholder link lengths the arm reaches every level of a stack only 25–39 cm from the base,
  hence 3 grid rows. IK picks the smallest feasible gripper tilt (50–66° on the grid).
- A camera in front of the arm cannot see cubes behind a stack. Those cubes are missing from `:init`.
- The background is a fixed image of the empty workspace: recapture it after lighting changes or if the
  camera moves.
- There is no closed-loop replanning. The executor works open-loop.
  However, the whole plan is checked for reachability before the arm moves, and the tracker is
  re-synchronised afterwards, so object IDs persist across missions.
