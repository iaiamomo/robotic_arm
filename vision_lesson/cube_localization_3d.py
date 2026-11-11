import cv2
import numpy as np
from common import (
    Mosaic,
    capture_background,
    colorize_depth,
    make_parser,
    open_source,
    put_lines,
    put_text,
    save_snapshot,
)
from vision_core import COLORS_BGR, get_mask_from_frames, identify_cubes

args = make_parser("Cube extraction with camera depth").parse_args()
src = open_source(args, ir=True)

W = "Cube extraction + depth"
view = Mosaic(W, cols=2, scale=0.75)
threshold = 50
diff_mode = "max"
margin = 10
aspect = (0.6, 1.4)
min_area = 900

depth_erosion = 3
depth_statistic = np.median
include_missing = False
depth_far = 1.2  # Maximum distance of the depth colormap, in meters.

background = None


def cube_depth(frame, contour):
    """Estimate depth; defaults exclude edges/zeros and use the median."""
    region = np.zeros(frame["depth"].shape, dtype=np.uint8)
    cv2.drawContours(region, [contour], -1, 255, -1)
    region = cv2.erode(region, np.ones((depth_erosion, depth_erosion), np.uint8))
    values = frame["depth"][region > 0]
    if not include_missing:
        values = values[values > 0]
    return float(depth_statistic(values)) * frame["depth_scale"] if values.size else None


while True:
    frame = src.get_frames()
    color = frame["color"]
    out = color.copy()
    mask = np.zeros(color.shape[:2], dtype=np.uint8)
    if background is None:
        put_lines(
            out,
            ["Empty the table and press 'b'"],
            color=(0, 255, 255),
        )
    else:
        mask, _, _ = get_mask_from_frames(background, color, threshold, diff_mode)
        cubes, rejected = identify_cubes(mask, color, min_area, aspect, margin, verbose=True)

        for r in rejected:
            cv2.drawContours(out, [r["contour"]], -1, (160, 160, 160), 1)
            x, y, w, h = r["bbox"]
            put_text(out, r["why"], (x, y + h + 14), 0.4, (180, 180, 180))
        for c in cubes:
            c["depth_m"] = cube_depth(frame, c["contour"])
            x, y, w, h = c["bbox"]
            col = COLORS_BGR[c["label"]]
            # draw contour, bounding box, center
            cv2.drawContours(out, [c["contour"]], -1, col, 1)
            cv2.rectangle(out, (x, y), (x + w, y + h), col, 2)
            cx, cy = map(int, c["center"])
            cv2.circle(out, (cx, cy), 4, (255, 255, 255), -1)
            depth_text = f"Depth: {c['depth_m']:.3f} m" if c["depth_m"] is not None else "Depth: unavailable"
            put_text(out, f"{c['label']} | {depth_text}", (x, max(15, y - 6)), 0.45)
        put_lines(
            out,
            [
                f"accepted: {len(cubes)}   rejected: {len(rejected)}",
                f"min_area={min_area}  aspect={aspect[0]:.2f}-{aspect[1]:.2f}  margin={margin}",
            ],
            color=(0, 255, 255),
        )

    depth_view = colorize_depth(frame["depth"], frame["depth_scale"], far=depth_far)
    if "ir" in frame:
        infrared = cv2.cvtColor(frame["ir"], cv2.COLOR_GRAY2BGR)
    else:
        infrared = np.zeros_like(color)
        put_lines(infrared, ["IR stream unavailable in this replay"])
    # The projected pattern is visible in IR; depth colors encode distance.
    view.show(
        [out, mask, depth_view, infrared],
        ["detections + camera depth", "foreground", "Depth (m): near = red, far = blue", "Infrared | e: toggle projector"],
    )

    k = cv2.waitKey(1) & 0xFF
    if k in (27, ord("q")):
        break
    elif k == ord("b"):
        background = capture_background(src, 10)
        print("background captured")
    elif k in (ord("c"), ord("n")):
        src.toggle_scene()
    elif k == ord("p") and background is not None:
        for c in cubes:
            depth_text = f"{c['depth_m']:.3f} m" if c["depth_m"] is not None else "unavailable"
            print(
                f"{c['label']:5s} center=({c['center'][0]:6.1f},{c['center'][1]:6.1f}) bbox={c['bbox']} "
                f"area={c['area']:.0f} aspect={c['aspect']:.2f} depth={depth_text}"
            )
        for r in rejected:
            print(f"  rejected {r['label']:5s} bbox={r['bbox']} reason: {r['why']}")
    elif k == ord("s"):
        save_snapshot(frame)
    elif k == ord("e"):
        enabled = src.toggle_emitter()
        print("IR projector unavailable" if enabled is None else f"IR projector: {'ON' if enabled else 'OFF'}")

src.close()
cv2.destroyAllWindows()


