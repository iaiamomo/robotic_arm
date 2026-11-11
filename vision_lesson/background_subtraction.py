import cv2
import numpy as np

from common import Mosaic, capture_background, make_parser, open_source, put_lines
from vision_core import color_masks, get_mask_from_frames

args = make_parser("Background subtraction and color extraction").parse_args()
src = open_source(args)
threshold = 50
diff_mode = "max"
opening = 3
margin = 10


view = Mosaic("RGB | Extracted colors", cols=2, scale=0.75)
background = None

try:
    while True:
        color = src.get_frames()["color"]
        rgb = color.copy()
        extracted = np.zeros_like(color)
        diff = np.zeros_like(color)
        foreground = np.zeros(color.shape[:2], dtype=np.uint8)

        if background is None:
            put_lines(rgb, ["Empty the table and press b to capture the background"])
        else:
            # Background difference selects changed regions.
            foreground, diff, _ = get_mask_from_frames(
                background, color, threshold, diff_mode, open_kernel=opening
            )
            # Read colors from RGB, restricted to foreground.
            masks = color_masks(color, margin)
            for label in ("RED", "GREEN", "BLUE"):
                mask = (masks[label] & (foreground > 0)).astype(np.uint8) * 255
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
                extracted[mask > 0] = color[mask > 0]

        view.show(
            [rgb, extracted, diff, foreground],
            ["RGB", "Extracted red, green and blue", "Absolute difference: background - current frame", "Foreground mask"],
        )
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            break
        if key == ord("b"):
            background = capture_background(src, n=10)
        elif key in (ord("c"), ord("n")):
            src.toggle_scene()
finally:
    src.close()
    cv2.destroyAllWindows()

