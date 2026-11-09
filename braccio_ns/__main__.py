"""Command line entry point.

  python -m braccio_ns gui        [--sim] [--goal G] [--no-preview]
  python -m braccio_ns mission "put the red block on the green one" [--sim] [--goal G] [--dry-run]
  python -m braccio_ns calibrate  [--sim]
  python -m braccio_ns marker     [--id 0] [--out marker_0.png]
  python -m braccio_ns reach      # IK reachability map of the grid
  python -m braccio_ns ik CELL_1_2 [--level 2] [--above]   # MOVE command for a cell (or X Y Z in metres)
  python -m braccio_ns grip 230 120 230   # gripper test (open, close, open)
Common: --config my.json  --ff /path/to/ff  --port COM3
"""

import argparse
import sys

from .config import load_config, load_dotenv


def make_cfg(a):
    over = {}
    if getattr(a, "ff", None):
        over.setdefault("planner", {}).update(backend="ff", command=[a.ff])
    if getattr(a, "port", None):
        over.setdefault("serial", {})["port"] = a.port
    return load_config(a.config, over)


def cmd_marker(a):
    import cv2
    import cv2.aruco as aruco

    img = aruco.generateImageMarker(aruco.getPredefinedDictionary(aruco.DICT_4X4_50), a.id, 400)
    cv2.imwrite(a.out, img)
    print(f"Marker {a.id} saved as {a.out}")


def cmd_ik(a):
    """Print the MOVE command that puts the grasp point at a position (no serial needed)."""
    import numpy as np

    from .kinematics import BraccioIK, Unreachable
    from .world import Grid

    cfg = make_cfg(a)
    r, cube = cfg["robot"], cfg["vision"]["cube_size_m"]
    if len(a.target) == 1:
        cell = a.target[0].upper()
        g = Grid(cfg["grid"])
        if cell not in g.cells():
            raise SystemExit(f"unknown cell {cell} (valid: {g.cells()[0]} .. {g.cells()[-1]})")
        x, y = g.center(cell)
        z = (a.level - 0.5) * cube + (r["approach_height_m"] if a.above else 0.0)
        what = f"{cell}, level {a.level}" + (" (approach, above)" if a.above else " (grasp)")
    elif len(a.target) == 3:
        x, y, z = (float(v) for v in a.target)
        what = "point"
    else:
        raise SystemExit("give X Y Z in metres, or a cell name such as CELL_1_2")
    ik = BraccioIK(r)
    try:
        sol = ik.solve(np.array([x, y, z]))
    except Unreachable as e:
        raise SystemExit(f"unreachable: {e}") from e
    grip = r["grip_open_deg"] if a.grip is None else a.grip
    fw = ik.q_to_firmware(sol["q"], grip)
    print(f"{what}: x={x:.3f} y={y:.3f} z={z:.3f} m, gripper tilt {sol['tilt_deg']:.0f} deg")
    print("MOVE " + " ".join(f"{v:.1f}" for v in fw))


def cmd_reach(a):
    from .kinematics import BraccioIK, Unreachable
    from .world import Grid

    cfg = make_cfg(a)
    ik, g, cube = BraccioIK(cfg["robot"]), Grid(cfg["grid"]), cfg["vision"]["cube_size_m"]
    h = cfg["robot"]["approach_height_m"]
    print("Max tilt (deg) to grasp at levels 1..3 (+approach); X = unreachable")
    for r in range(g.rows):
        row = []
        for c in range(g.cols):
            x, y = g.center(g.name(r, c))
            worst = 0
            for lvl in (1, 2, 3):
                for z in ((lvl - 0.5) * cube, (lvl - 0.5) * cube + h):
                    try:
                        worst = max(worst, abs(ik.solve([x, y, z])["tilt_deg"]))
                    except Unreachable:
                        worst = None
                        break
                if worst is None:
                    break
            row.append(" X " if worst is None else f"{worst:3.0f}")
        print(f"row {r}: " + " ".join(row))


def cmd_calibrate(a):
    from .system import System

    s = System(make_cfg(a), sim=a.sim, goal="(:goal (and))")
    try:
        s.calibrate()
    finally:
        s.shutdown()


def cmd_grip(a):
    """Gripper test without the GUI (close the Arduino serial monitor first)."""
    from .serial_link import BraccioSerial

    s = make_cfg(a)["serial"]
    arm = BraccioSerial(s["port"], s["baudrate"], s["timeout_s"], s["move_settle_s"])
    try:
        for g in a.angles:
            print(f"GRIP {g:.1f} -> {arm.grip(g)}")
    finally:
        arm.close()


def cmd_mission(a):
    from .system import System

    cfg = make_cfg(a)
    s = System(cfg, sim=a.sim, goal=a.goal)
    s.capture_background()
    s.warmup(15)
    mem = s.perception.target_memory()
    print("Detected:", {k: (v["level"], v["on_top_of"]) for k, v in mem.items()})
    res = s.run_mission(a.request, execute=not a.dry_run)
    print(res)
    if a.sim and res.ok and not a.dry_run:
        s.warmup(15)
        print("After execution:", {k: (v["level"], v["on_top_of"]) for k, v in s.perception.target_memory().items()})
    s.shutdown()
    return 0 if res.ok else 1


def cmd_gui(a):
    from .gui import App
    from .system import System

    cfg = make_cfg(a)
    s = System(cfg, sim=a.sim, goal=a.goal)
    if a.sim:
        s.capture_background()
    App(s, preview=not a.no_preview).run()


def main(argv=None):
    load_dotenv()  # e.g. OPENROUTER_API_KEY
    p = argparse.ArgumentParser(prog="braccio_ns")
    p.add_argument("--config")
    p.add_argument("--ff", help="use the FF binary at this path instead of Fast Downward")
    p.add_argument("--port", help="serial port of the Braccio++")
    sub = p.add_subparsers(dest="cmd", required=True)

    m = sub.add_parser("marker")
    m.add_argument("--id", type=int, default=0)
    m.add_argument("--out", default="marker_0.png")
    m.set_defaults(fn=cmd_marker)
    sub.add_parser("reach").set_defaults(fn=cmd_reach)
    k = sub.add_parser("ik", help="MOVE command for a point (X Y Z metres) or a cell (CELL_r_c)")
    k.add_argument("target", nargs="+")
    k.add_argument("--level", type=int, default=1, help="stack level for a cell (1 = on the table)")
    k.add_argument("--above", action="store_true", help="approach pose, approach_height_m higher")
    k.add_argument("--grip", type=float, help="gripper angle (default: grip_open_deg)")
    k.set_defaults(fn=cmd_ik)
    gr = sub.add_parser("grip", help="send GRIP commands, e.g. grip 230 120 230")
    gr.add_argument("angles", type=float, nargs="+")
    gr.set_defaults(fn=cmd_grip)
    c = sub.add_parser("calibrate")
    c.add_argument("--sim", action="store_true")
    c.set_defaults(fn=cmd_calibrate)
    for name, fn in (("mission", cmd_mission), ("gui", cmd_gui)):
        q = sub.add_parser(name)
        if name == "mission":
            q.add_argument("request")
            q.add_argument("--dry-run", action="store_true", help="plan only, do not move the arm")
        else:
            q.add_argument("--no-preview", action="store_true")
        q.add_argument("--sim", action="store_true", help="synthetic camera + simulated arm")
        q.add_argument("--goal", help="skip the LLM and use this PDDL goal")
        q.set_defaults(fn=fn)
    a = p.parse_args(argv)
    return a.fn(a) or 0


if __name__ == "__main__":
    sys.exit(main())
