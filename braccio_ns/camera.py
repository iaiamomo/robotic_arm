"""Intel RealSense D435i wrapper: colour + depth aligned to colour, deprojection."""
import numpy as np


class RealSenseCamera:
    def __init__(self, width=640, height=480, fps=30):
        import pyrealsense2 as rs
        self.rs = rs
        self.pipeline = rs.pipeline()
        conf = rs.config()
        conf.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        conf.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        profile = self.pipeline.start(conf)
        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        self.align = rs.align(rs.stream.color)
        for _ in range(30):  # let auto-exposure settle
            self.pipeline.wait_for_frames()

    def get_frames(self):
        frames = self.align.process(self.pipeline.wait_for_frames())
        color, depth = frames.get_color_frame(), frames.get_depth_frame()
        intr = color.profile.as_video_stream_profile().intrinsics
        return {
            "color": np.asanyarray(color.get_data()).copy(),
            "depth": np.asanyarray(depth.get_data()).copy(),
            "intrinsics": intr,
            "depth_scale": self.depth_scale,
        }

    def deproject(self, frame, uv, patch=5):
        """Pixel (u, v) -> 3D point (m) in the camera optical frame, using the median
        depth of a (2*patch+1)^2 window to reject holes/noise."""
        u, v = int(round(uv[0])), int(round(uv[1]))
        d = frame["depth"]
        win = d[max(v - patch, 0):v + patch + 1, max(u - patch, 0):u + patch + 1].astype(float)
        win = win[win > 0]
        if win.size == 0:
            return None
        dist = float(np.median(win)) * frame["depth_scale"]
        return np.asarray(self.rs.rs2_deproject_pixel_to_point(frame["intrinsics"], [u, v], dist))

    def close(self):
        self.pipeline.stop()


class PinholeIntrinsics:
    """Minimal stand-in for rs.intrinsics (used by SimCamera)."""

    def __init__(self, fx, fy, ppx, ppy, width, height):
        self.fx, self.fy, self.ppx, self.ppy, self.width, self.height = fx, fy, ppx, ppy, width, height


def deproject_pinhole(intr, uv, dist):
    """Same model as rs2_deproject_pixel_to_point with no distortion."""
    x = (uv[0] - intr.ppx) / intr.fx
    y = (uv[1] - intr.ppy) / intr.fy
    return np.array([x * dist, y * dist, dist])
