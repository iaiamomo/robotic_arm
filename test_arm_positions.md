# Braccio++ movement tests

How to move the arm by hand with serial commands, and how to turn a position on the table into the angles to send.

## 1. Sending commands

- **Arduino IDE serial monitor:** 115200 baud, line ending "Newline".
- **From a terminal**, for the gripper only: `python -m braccio_ns --port COM3 grip 200 150 200`.

Only one program at a time can open the serial port. Close the GUI before using the serial monitor, and vice versa.

The firmware replies `OK: ...` or `ERROR: ...`. If it adds ` (clamped)`, a value was out of range and was limited.

## 2. The three commands

| Command | What it does |
|---|---|
| `HOME` | Goes to the home pose `160 150 157.5 157.5 157.5 95`. **It also moves the gripper to 160.** |
| `GRIP g` | Moves **only** the gripper to angle `g`. The rest of the arm stays still. |
| `MOVE g r p e s b` | Moves all six joints together. Angles are in degrees, in this order: |

| Pos. | Letter | Joint | Firmware limits | Notes |
|---|---|---|---|---|
| 1 | `g` | gripper | 120 – 230 | **200 = fully open, 150 = grasp.** Do not use 120 or 230: the servo stalls and the board has to be power-cycled. |
| 2 | `r` | wrist roll | 0 – 315 | Fixed at **150** throughout the code. |
| 3 | `p` | wrist pitch | 80 – 230 | 157.5 = in line with the forearm |
| 4 | `e` | elbow | 75 – 225 | 157.5 = in line with the upper arm |
| 5 | `s` | shoulder | 60 – 250 | 157.5 = arm straight up |
| 6 | `b` | base | 0 – 315 | **95 = straight towards the grid** (the HOME direction). Larger angles rotate to the left: 140 is about 45° to the left. 1° of command = 1° of rotation. |

With every joint at 157.5 the arm is straight up. HOME is exactly this pose, with the base turned towards the grid.

## 3. Coordinates

The code works in metres, in the **robot frame**:

```
            ▲ x  (forward, towards the grid = HOME direction)
            │
   y ◄──────●  origin: on the base axis, at the level of the table where the cubes rest
  (left)           z: up (z = 0 is that table; the arm base stands 2.4 cm higher, on a platform)
```

**The grid** (3 rows × 5 columns, `CELL_row_column`) seen from above:

```
             col 4    col 3    col 2    col 1    col 0
           ┌────────┬────────┬────────┬────────┬────────┐
  row 2    │CELL_2_4│CELL_2_3│CELL_2_2│CELL_2_1│CELL_2_0│  x = 36 cm
           ├────────┼────────┼────────┼────────┼────────┤
  row 1    │CELL_1_4│CELL_1_3│CELL_1_2│CELL_1_1│CELL_1_0│  x = 31 cm
           ├────────┼────────┼────────┼────────┼────────┤
  row 0    │CELL_0_4│CELL_0_3│CELL_0_2│CELL_0_1│CELL_0_0│  x = 26 cm
           └────────┴────────┴───┬────┴────────┴────────┘
    y =      +12      +6       0 │     −6       −12   (cm, cell centres)
                                 │  HOME direction
                               [base]
```

Cells are 5 cm deep and 6 cm wide. Cubes are 5 cm: a cube on the table has its centre at z = 2.5 cm, a cube on top of another at z = 7.5 cm, a third one at z = 12.5 cm.

### From coordinates to angles: the `ik` command

`MOVE` takes angles, not coordinates. The `ik` command computes the `MOVE` for a cell or a point, without moving the arm:

```bash
python -m braccio_ns ik CELL_0_2 --above         # 6 cm above the cube in CELL_0_2 (approach)
python -m braccio_ns ik CELL_0_2                 # grasp point of a cube on the table in CELL_0_2
python -m braccio_ns ik CELL_2_2 --level 2       # cube on level 2 (on top of another) in CELL_2_2
python -m braccio_ns ik 0.30 0 0.10              # any point: x y z in metres
python -m braccio_ns ik CELL_0_2 --grip 150      # same pose with the gripper closed
```

Example output:
```
CELL_0_2, level 1 (grasp): x=0.260 y=0.000 z=0.015 m, gripper tilt 24 deg
MOVE 200.0 150.0 226.4 217.0 185.1 95.0
```
Paste the `MOVE` line into the serial monitor. If the point is out of reach, the command says so ("unreachable"). The arm reaches every height of a stack only between about 25 and 39 cm from the base.

## 4. Example: picking and placing a cube by hand

Cube in `CELL_0_2`, to be moved to `CELL_1_4`. This is the same sequence the program uses:

```
HOME
GRIP 200                                  ← hand fully open, arm still
MOVE 200 150 228.0 217.8 170.7 95.0       ← above the cube (ik CELL_0_2 --above)
MOVE 200 150 226.4 217.0 185.1 95.0       ← down onto the cube (ik CELL_0_2)
GRIP 150                                  ← close
MOVE 150 150 228.0 217.8 170.7 95.0       ← lift with the cube
MOVE 150 150 157.5 157.5 157.5 95.0       ← back home holding the cube (not HOME: it would open the hand)
MOVE 150 150 226.5 181.4 200.6 116.2      ← above CELL_1_4 (ik CELL_1_4 --above --grip 150)
MOVE 150 150 229.8 173.0 215.7 116.2      ← down in CELL_1_4 (ik CELL_1_4 --grip 150)
GRIP 200                                  ← release
MOVE 200 150 226.5 181.4 200.6 116.2      ← lift (ik CELL_1_4 --above)
MOVE 200 150 157.5 157.5 157.5 95.0       ← back home with the hand open
```

These values depend on the link lengths in the URDF and on the current configuration. Always regenerate them with `ik` instead of copying them from here.

## 5. To check: shoulder and elbow direction

The code assumes that, for shoulder, elbow and wrist pitch, **angles above 157.5 bend the arm forward** (towards the grid), with `joint_signs` = 1 in `config/default.json`. That is why the `MOVE` commands computed by `ik` have shoulder and elbow above 157.5.

The original example below ("lean forward a little" with shoulder and elbow at **140**) suggests the opposite. Test one joint at a time, starting from HOME:

```
MOVE 200 150 157.5 157.5 175 95           ← shoulder only at 175: does the arm go forward or backward?
MOVE 200 150 157.5 175 157.5 95           ← elbow only at 175
MOVE 200 150 175 157.5 157.5 95           ← wrist pitch only at 175
```

If a joint goes **backward** (away from the grid), set its `joint_signs` to `-1` in `config/default.json`. Otherwise the computed movements go the wrong way.

## 6. Measuring the droop (`robot.droop_m`)

Far from the base the real arm ends up lower than the model says. The servos sag under the load of the extended arm, and the URDF link lengths are approximate. During calibration this is why the gripper touches the table at the farthest points. The IK corrects it with `robot.droop_m`, which has to be measured once:

1. Attach the calibration marker (its weight is part of the load) and send `HOME`.
2. Send the `MOVE` commands below one at a time. They point straight ahead (y = 0), gripper closed at 150 as in calibration, with the grasp point commanded **10 cm** above the table.
3. With a ruler, measure the height of the grasp point (the point between the fingers) above the table. drop = 0.10 − measured height (metres).

| Reach | Command | Measured height | Drop |
|---|---|---|---|
| 26 cm | `MOVE 150 150 225.9 205.1 169.2 95.0` | | |
| 30 cm | `MOVE 150 150 225.4 185.3 185.5 95.0` | | |
| 34 cm | `MOVE 150 150 216.4 174.9 196.9 95.0` | | |
| 38 cm | `MOVE 150 150 204.3 164.2 209.7 95.0` | | |

4. Write the `[reach, drop]` pairs, in metres, in `config/default.json`. For example (made-up values): `"droop_m": [[0.26, 0.005], [0.30, 0.012], [0.34, 0.025], [0.38, 0.045]]`.
5. Check: `python -m braccio_ns ik 0.38 0 0.10 --grip 150` now prints a higher pose. Send it: the grasp point should be at 10 cm.

Notes:
- Between the measured reaches the drop is interpolated. Beyond them the closest value is used, so measure up to 38 cm (the far corners of the grid).
- The commands in the table are for an empty `droop_m`. If you change the URDF or `joint_offsets_deg`, regenerate them with `ik X 0 0.10 --grip 150` while `droop_m` is empty, then measure again.

## 7. Original examples

HOME written as a MOVE (gripper at 160, like the firmware HOME):

    MOVE 160 150 157.5 157.5 157.5 95

Rotate only the base by about 45° to the left:

    MOVE 160 150 157.5 157.5 157.5 140

Lean forward a little (see section 5 for the shoulder and elbow direction):

    MOVE 160 150 157.5 140 140 95
