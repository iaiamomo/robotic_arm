"""Vision functions extracted from braccio_ns/vision.py for teaching.

RGB + background -> get_mask_from_frames -> foreground mask.
RGB -> color_masks -> red, green, and blue masks.
Foreground + color masks -> identify_cubes -> 2D cube candidates.
Depth + calibration -> localize -> [x, y, z_top] in the robot frame.
drop_occluded removes visible side faces of covered cubes.

Teaching additions: verbose=True returns rejected candidates and reasons;
localize also returns the top-surface points for visualization.
"""

import cv2
import numpy as np

COLORS_BGR = {"RED": (0, 0, 255), "GREEN": (0, 200, 0), "BLUE": (255, 0, 0)}


# =================================================================== 1. background subtraction
def get_mask_from_frames(background, frame, threshold=50, mode="max", open_kernel=3):
    """Foreground mask: pixels that differ sufficiently from the background.

    Compute abs(background - frame) per BGR channel, then reduce to one value:
      gray: weighted luminance (0.114 B + 0.587 G + 0.299 R).
      max: maximum channel difference (the main pipeline default).
    Blue has low luminance, so gray mode may miss saturated blue objects.
    Morphological opening (erosion then dilation) removes isolated noise.
    """
    diff = cv2.absdiff(background, frame)
    diff_1ch = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY) if mode == "gray" else diff.max(axis=2)
    mask = cv2.inRange(diff_1ch, threshold, 255)
    if open_kernel > 0:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((open_kernel, open_kernel), np.uint8))
    return mask, diff, diff_1ch


# =================================================================== 2. color
def classify_color(mean_bgr, margin=10):
    """Chromatic dominance: a channel must exceed both others by margin.
    This rule assumes saturated red, green, and blue objects."""
    b, g, r = mean_bgr
    if b > g + margin and b > r + margin:
        return "BLUE"
    if g > b + margin and g > r + margin:
        return "GREEN"
    if r > b + margin and r > g + margin:
        return "RED"
    return None


def color_masks(frame, margin=10):
    """Apply classify_color's rule to every pixel using NumPy.
    int16 prevents uint8 overflow when adding margin."""
    b, g, r = [frame[..., i].astype(np.int16) for i in range(3)]
    return {
        "BLUE": (b > g + margin) & (b > r + margin),
        "GREEN": (g > b + margin) & (g > r + margin),
        "RED": (r > b + margin) & (r > g + margin),
    }


# =================================================================== 3. cube extraction
def identify_cubes(mask, frame, min_area=900, aspect=(0.6, 1.4), margin=10, verbose=False):
    """Find cube candidates from color AND foreground masks.

    A tilted view of a red-on-blue stack may form one foreground blob.
    Split it by color, apply 5x5 opening, then filter each contour by:
      - area > min_area;
      - aspect[0] < bounding-box width / height < aspect[1].
    Compute the 2D centroid from contour moments: (m10/m00, m01/m00).
    """
    results, rejected = [], []
    for label, cm in color_masks(frame, margin).items():
        m = (cm & (mask > 0)).astype(np.uint8) * 255  # Color AND foreground
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)
            ar = w / h
            if area > min_area and aspect[0] < ar < aspect[1]:
                mo = cv2.moments(contour)
                results.append(
                    {
                        "label": label,
                        "bbox": (x, y, w, h),
                        "center": (mo["m10"] / mo["m00"], mo["m01"] / mo["m00"]),
                        "contour": contour,
                        "area": area,
                        "aspect": ar,
                    }
                )
            elif verbose and area > 30:  # Ignore tiny contours even in the rejected list.
                why = f"area {area:.0f}<={min_area}" if area <= min_area else f"aspect {ar:.2f}"
                rejected.append({"label": label, "bbox": (x, y, w, h), "contour": contour, "why": why})
    return (results, rejected) if verbose else results


# =================================================================== 4. depth to 3D
def deproject_pixels(intr, us, vs, depth_m):
    """Inverse pinhole model: pixel + depth -> camera optical-frame point.
        X = (u - ppx) / fx * Z      Y = (v - ppy) / fy * Z      Z = depth
    Matches rs2_deproject_pixel_to_point for the D435 color stream (~zero distortion)."""
    x = (us - intr.ppx) / intr.fx
    y = (vs - intr.ppy) / intr.fy
    return np.stack([x * depth_m, y * depth_m, depth_m], axis=1)


def project_to_image(intr, pts_cam):
    """Pinhole projection: camera points (N, 3) -> pixels (N, 2) and depths."""
    pts = np.asarray(pts_cam, float)
    z = pts[:, 2]
    uv = np.stack([intr.fx * pts[:, 0] / z + intr.ppx, intr.fy * pts[:, 1] / z + intr.ppy], axis=1)
    return uv, z


def localize(frame, contour, cam_to_robot, top_band=0.006):
    """Estimate cube position [x, y, z_top] in the robot frame.

    A tilted view includes the top and front faces, shifting the 2D centroid.
      1. Erode the blob by one pixel to remove mixed-depth edges.
      2. Deproject valid pixels and transform them into the robot frame.
      3. Estimate z_top as the 95th percentile of heights.
      4. Average x and y of points above z_top - top_band.
    Return (position, top_points_robot), or (None, None) for insufficient depth.
    """
    mask = np.zeros(frame["depth"].shape, np.uint8)
    cv2.drawContours(mask, [contour], -1, 1, -1)
    mask = cv2.erode(mask, np.ones((3, 3), np.uint8))
    vs, us = np.nonzero(mask)
    d = frame["depth"][vs, us].astype(float) * frame["depth_scale"]
    ok = d > 0  # Zero depth means no stereo correspondence.
    if ok.sum() < 20:
        return None, None
    pc = deproject_pixels(frame["intrinsics"], us[ok], vs[ok], d[ok])
    pr = cam_to_robot(pc)
    z_top = np.percentile(pr[:, 2], 95)
    top = pr[pr[:, 2] > z_top - top_band]
    return np.array([top[:, 0].mean(), top[:, 1].mean(), z_top]), top


def drop_occluded(dets, cube):
    """Remove a component whose top is clearly below another overlapping in xy.
    It is a visible side face of a covered cube (HIDDEN_UNDER_* in the planner)."""
    keep = []
    for d in dets:
        covered = any(
            o is not d
            and o["pos"][2] > d["pos"][2] + cube / 2
            and np.linalg.norm(np.subtract(o["pos"][:2], d["pos"][:2])) < cube
            for o in dets
        )
        if not covered:
            keep.append(d)
    return keep


# =================================================================== drawing
def draw_grid(img, grid, tf, intr, color=(230, 230, 230)):
    """Project table-plane grid cells (robot-frame z = 0) onto the image."""
    hr, hc = grid.row_step / 2, grid.col_step / 2
    for cell in grid.cells():
        c = grid.center(cell)
        xy = [c + a * hr + b * hc for a, b in ((-1, -1), (-1, 1), (1, 1), (1, -1))] + [c]
        uv, z = project_to_image(intr, tf.to_camera([[x, y, 0.0] for x, y in xy]))
        if (z <= 0).any():
            continue
        cv2.polylines(img, [np.round(uv[:4]).astype(np.int32)], True, color, 1, cv2.LINE_AA)
        label = cell.removeprefix("CELL_")
        cv2.putText(img, label, (int(uv[4, 0]) - 8, int(uv[4, 1]) + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)
