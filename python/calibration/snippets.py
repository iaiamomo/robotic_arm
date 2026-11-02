"""Eye-to-hand calibration — Listings 3.3 and 3.5 (one-line fragments).

The full calibration routine (33 reference points, 20 frames per point,
X_mat / Y_mat accumulation, Umeyama similarity estimation) is described in
Section 3.2.2 but its code is NOT included in the thesis.
"""
import cv2.aruco as aruco
import pyrealsense2 as rs

# Listing 3.3 — ArUco detection
detector = aruco.ArucoDetector(
    aruco.getPredefinedDictionary(aruco.DICT_4X4_50),
    aruco.DetectorParameters()
)


def deproject(intr, cx, cy, dist):
    # Listing 3.5 — Deprojection of 2D pixel coordinates to 3D camera frame
    p_3d = rs.rs2_deproject_pixel_to_point(intr, [cx, cy], dist)
    return p_3d


# TODO (missing from thesis): Umeyama estimation of (R, t, s) between X_mat and Y_mat
