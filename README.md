# Braccio++ Neuro-Symbolic Manipulation

Code from the Master's thesis of **Agnese Maria Capparelli** (Sapienza Università di Roma):
natural-language control of an Arduino Braccio++ arm through an LLM → PDDL → Fast-Forward → serial pipeline,
with Intel RealSense D435i perception.

> **Provenance.** Every file here is transcribed from the code listings in the thesis PDF.
> Only the Arduino firmware is complete. The Python files are **fragments**. The central orchestrator
> and several modules are not in the thesis (see "Missing components"). Function bodies are left
> verbatim, including bugs; see [`NOTES.md`](NOTES.md).

## Pipeline

```
RealSense frames ──► vision (bg subtraction + colour) ──► target_memory ──► PDDL :init
User NL command ──► LLM (Llama 3.2 via LM Studio, T=0) ─────────────────► PDDL :goal
domain.pddl + problem.pddl ──► FF planner (WSL) ──► plan ──► IK (IKPy) ──► serial ──► Braccio++ firmware
```

## Layout

| Path | Thesis source | Status |
|---|---|---|
| `firmware/braccio_serial/braccio_serial.ino` | Listing 3.1 | complete |
| `pddl/domain.pddl` | Listing 3.8 | complete |
| `pddl/problem_example.pddl` | Listing 3.9 | example |
| `python/vision/generate_aruco_marker.py` | Listing 3.4 | complete |
| `python/vision/detection.py` | Listings 3.6, 3.7 | functions only |
| `python/calibration/snippets.py` | Listings 3.3, 3.5 | one-liners |
| `python/kinematics/ik_chain.py` | Listing 3.2 | chain init only |
| `python/planning/pddl_problem.py` | Listings 3.10, 3.11 | functions only, has bugs |
| `python/planning/planner.py` | Listings 3.12, 3.13 | functions only |
| `python/llm/prompt.py` | §3.5 system prompt | prompt text only |

## Firmware serial protocol (115200 baud)

```
HOME
GRIP <angle>                      # clamped to [120, 230]
MOVE <grip> <roll> <pitch> <elbow> <shoulder> <base>
```
Requires the `Arduino_Braccio_plusplus` library (Arduino IDE).

## Missing components (not in the thesis)

- `start_mission` orchestrator and plan-to-joint-angle execution
- Tkinter (or similar) GUI described in §3.1
- Real-time vision loop, cube ID assignment, `target_memory` population, stacking (`level`, `on_top_of`) detection
- Full eye-to-hand calibration (33 points × 20 frames, Umeyama transform)
- `braccio.urdf`
- LM Studio client call and the few-shot examples
- Serial client (pyserial) on the Python side
- FF binary setup under WSL

## Requirements

Python ≥ 3.12 (the f-string in `save_problem_file` uses `"\n"` inside a replacement field), see `requirements.txt`.
