import cv2
import numpy as np
from common import (
    Mosaic,
    capture_background,
    make_parser,
    open_source,
    put_lines,
    put_text,
    save_snapshot,
)
from vision_core import COLORS_BGR, get_mask_from_frames, identify_cubes

args = make_parser("Cube extraction").parse_args()
src = open_source(args)

W = "Cube extraction"
view = Mosaic(W, cols=2, scale=0.75)
threshold = 50
diff_mode = "max"
margin = 10
aspect = (0.6, 1.4)
min_area = 900


background = None

while True:
    frame = src.get_frames()
    color = frame["color"]
    out = color.copy()
    if background is None:
        put_lines(
            out,
            ["Empty the table and press 'b'"],
            color=(0, 255, 255),
        )
        view.show([out, np.zeros_like(out)])
    else:
        mask, _, _ = get_mask_from_frames(background, color, threshold, diff_mode)
        cubes, rejected = identify_cubes(mask, color, min_area, aspect, margin, verbose=True)

        for r in rejected:
            cv2.drawContours(out, [r["contour"]], -1, (160, 160, 160), 1)
            x, y, w, h = r["bbox"]
            put_text(out, r["why"], (x, y + h + 14), 0.4, (180, 180, 180))
        for c in cubes:
            x, y, w, h = c["bbox"]
            col = COLORS_BGR[c["label"]]
            # draw contour, bounding box, center
            cv2.drawContours(out, [c["contour"]], -1, col, 1)
            cv2.rectangle(out, (x, y), (x + w, y + h), col, 2)
            cx, cy = map(int, c["center"])
            cv2.circle(out, (cx, cy), 4, (255, 255, 255), -1)
            put_text(out, f"{c['label']} A={c['area']:.0f} ar={c['aspect']:.2f}", (x, y - 6), 0.42)
        put_lines(
            out,
            [
                f"accepted: {len(cubes)}   rejected: {len(rejected)}",
                f"min_area={min_area}  aspect={aspect[0]:.2f}-{aspect[1]:.2f}  margin={margin}",
            ],
            color=(0, 255, 255),
        )

        view.show(
            [out, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)],
            ["result", "foreground (a stack may form one blob)"],
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
            print(
                f"{c['label']:5s} center=({c['center'][0]:6.1f},{c['center'][1]:6.1f}) bbox={c['bbox']} "
                f"area={c['area']:.0f} aspect={c['aspect']:.2f}"
            )
        for r in rejected:
            print(f"  rejected {r['label']:5s} bbox={r['bbox']} reason: {r['why']}")
    elif k == ord("s"):
        save_snapshot(frame)

src.close()
cv2.destroyAllWindows()

