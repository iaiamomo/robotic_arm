import cv2
from common import Mosaic, colorize_depth, make_parser, open_source, put_lines, save_snapshot

p = make_parser("RGB and depth overlay")
args = p.parse_args()
align_depth = True
blend = 0.5
far = 1.2

src = open_source(args, align=align_depth, ir=True)

view = Mosaic("RGB + Depth | Infrared", cols=2)
print("Keys: e = toggle IR projector, s = snapshot, q / ESC = quit")

try:
    while True:
        frame = src.get_frames()
        color, depth, scale = frame["color"], frame["depth"], frame["depth_scale"]
        # RealSense aligns depth to RGB by default.
        dcol = colorize_depth(depth, scale, far=far)
        display_depth = depth
        if depth.shape != color.shape[:2]:
            size = (color.shape[1], color.shape[0])
            dcol = cv2.resize(dcol, size, interpolation=cv2.INTER_NEAREST)
            display_depth = cv2.resize(depth, size, interpolation=cv2.INTER_NEAREST)

        overlay = cv2.addWeighted(color, 1 - blend, dcol, blend, 0)
        overlay[display_depth == 0] = color[display_depth == 0]

        if view.hover is not None:
            panel, u, v = view.hover
            # IR has its own viewpoint; its pixels are not aligned to RGB.
            if panel == 0 and 0 <= v < display_depth.shape[0] and 0 <= u < display_depth.shape[1]:
                raw = int(display_depth[v, u])
                lines = [f"(u, v) = ({u}, {v})"]
                lines.append(f"Depth: {raw * scale:.3f} m" if raw else "Depth: unavailable")
                cv2.drawMarker(overlay, (u, v), (255, 255, 255), cv2.MARKER_CROSS, 16, 1)
                put_lines(overlay, lines, color=(255, 255, 0))

        if "ir" in frame:
            infrared = cv2.cvtColor(frame["ir"], cv2.COLOR_GRAY2BGR)
        else:
            infrared = overlay * 0
            put_lines(infrared, ["IR stream unavailable in this replay"])
        view.show([overlay, infrared], ["RGB + depth", "Infrared | e: toggle projector"])
        k = cv2.waitKey(1) & 0xFF
        if k in (27, ord("q")):
            break
        elif k == ord("s"):
            save_snapshot(frame)
        elif k == ord("e"):
            enabled = src.toggle_emitter()
            print("IR projector unavailable" if enabled is None else f"IR projector: {'ON' if enabled else 'OFF'}")
        elif k in (ord("c"), ord("n")):
            src.toggle_scene()
finally:
    src.close()
    cv2.destroyAllWindows()

