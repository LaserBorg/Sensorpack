import unittest
from unittest.mock import patch
import threading

import numpy as np
import open3d as o3d

from ToF.colorize import RollingToFAverage, main, read_tof_frame


class Frame:
    def __init__(self, depth, amplitude):
        self.depth_data = np.array(depth, dtype=np.float32)
        self.confidence_data = np.array(amplitude, dtype=np.float32)


class Camera:
    def __init__(self, frames):
        self.frames = list(frames)
        self.released = []

    def requestFrame(self, timeout):
        return self.frames.pop(0) if self.frames else None

    def releaseFrame(self, frame):
        self.released.append(frame)


class CaptureTests(unittest.TestCase):
    def test_rolling_average_excludes_invalid_depth_and_evicts_old_frames(self):
        frames = [
            Frame([[100, 0], [200, 300]], [[10, 20], [30, 40]]),
            Frame([[200, 400], [np.nan, 500]], [[30, 40], [50, 60]]),
            Frame([[300, 600], [400, 700]], [[50, 60], [70, 80]]),
        ]
        cam = Camera(frames)
        average = RollingToFAverage(2)

        with patch("ToF.colorize.ac.DepthData", Frame):
            self.assertTrue(read_tof_frame(cam, average))
            self.assertIsNone(average.mean())
            self.assertTrue(read_tof_frame(cam, average))
            depth, amplitude = average.mean()
            np.testing.assert_allclose(depth, [[150, 400], [200, 400]])
            np.testing.assert_allclose(amplitude, [[20, 30], [40, 50]])
            self.assertTrue(read_tof_frame(cam, average))

        depth, amplitude = average.mean()
        np.testing.assert_allclose(depth, [[250, 500], [400, 600]])
        np.testing.assert_allclose(amplitude, [[40, 50], [60, 70]])
        self.assertEqual(cam.released, frames)

    def test_missing_frame_does_not_change_average(self):
        cam = Camera([])
        average = RollingToFAverage(20)

        self.assertFalse(read_tof_frame(cam, average))
        self.assertIsNone(average.mean())

    def test_live_view_updates_and_saves_latest_cloud(self):
        rgb_ready = threading.Event()
        rgb_frame = np.full((2, 2, 3), (0, 0, 255), dtype=np.uint8)

        class RGBCamera:
            def __init__(self, cam_id):
                self.closed = False

            def create_video_configuration(self, main):
                return main

            def configure(self, config):
                pass

            def set_controls(self, controls):
                pass

            def start(self):
                pass

            def autofocus_cycle(self, wait=True):
                return True

            def capture_array(self):
                rgb_ready.set()
                return rgb_frame

            def close(self):
                self.closed = True

        class ToFCamera(Camera):
            def __init__(self):
                super().__init__([Frame([[1000] * 2] * 2, [[0, 10], [20, 30]]) for _ in range(3)])
                self.closed = False

            def open(self, connection, cam_id):
                return 0

            def start(self, frame_type):
                return 0

            def getControl(self, control):
                return 4000

            def requestFrame(self, timeout):
                if not rgb_ready.wait(1):
                    raise AssertionError("RGB stream did not start")
                return super().requestFrame(timeout)

            def close(self):
                self.closed = True

        class Viewer:
            def __init__(self):
                self.updates = 0
                self.polls = 0
                self.cloud = None
                self.geometry = []

            def add_geometry(self, cloud):
                self.geometry.append(cloud)
                self.cloud = cloud

            def register_key_callback(self, key, callback):
                pass

            def update_geometry(self, cloud):
                self.updates += 1

            def poll_events(self):
                self.polls += 1
                return self.polls < 3

            def update_renderer(self):
                pass

            def destroy_window(self):
                pass

        viewer = Viewer()
        tof = ToFCamera()
        rgb = RGBCamera(0)
        intrinsic = o3d.camera.PinholeCameraIntrinsic(2, 2, 1, 1, 0, 0)
        matrix = np.eye(3)
        frustum = object()
        written = []
        with patch("ToF.colorize.Picamera2", return_value=rgb), \
            patch("ToF.colorize.ac.ArducamCamera", return_value=tof), patch("ToF.colorize.ac.DepthData", Frame), \
            patch("ToF.colorize.get_intrinsic_driver", return_value=intrinsic), \
            patch("ToF.colorize.load_calibration", return_value=(matrix, np.zeros(5), matrix, np.zeros(3), (2, 2))), \
            patch("ToF.colorize.cv2.getOptimalNewCameraMatrix", return_value=(matrix, None)), \
            patch("ToF.colorize.create_visualizer", return_value=viewer), \
            patch("ToF.colorize.create_frustum", return_value=frustum), \
            patch("ToF.colorize.apply_default_view"), patch("ToF.colorize.o3d.io.write_point_cloud", side_effect=lambda path, cloud, **kwargs: written.append(len(cloud.points))), \
            patch("ToF.colorize.cv2.imwrite", return_value=True), \
            patch("ToF.colorize.cv2.namedWindow"), patch("ToF.colorize.cv2.imshow"), \
            patch("ToF.colorize.cv2.waitKey", return_value=-1), patch("ToF.colorize.cv2.destroyAllWindows"), \
            patch("sys.argv", ["colorize.py", "--frames", "2"]):
            main()

        self.assertIs(viewer.geometry[0], frustum)
        self.assertIs(viewer.geometry[1], viewer.cloud)
        self.assertGreaterEqual(viewer.updates, 2)
        self.assertGreater(len(viewer.cloud.points), 0)
        self.assertEqual(written, [len(viewer.cloud.points)])
        self.assertEqual(len(tof.released), 3)
        self.assertTrue(tof.closed and rgb.closed)


if __name__ == "__main__":
    unittest.main()