'''
Continuously colorize a rolling average of ToF frames with live RGB.
Press s to save the current cloud and RGB image, f to refocus, or q to quit.
The latest cloud is also saved on exit. Keep the scene still while averaging.

Usage:
    python ToF/colorize.py [--rgb-id 0] [--tof-id 8] [--frames 20] [--save ToF/output/pcd_colorized.ply]
'''

import argparse
from collections import deque
import os
import threading

import cv2
import numpy as np
import open3d as o3d
from picamera2 import Picamera2

import sys
sys.path.insert(0, os.path.dirname(__file__))
from lib.depth_utils import (  # noqa: E402
    get_intrinsic_driver, convert_distance_to_zdepth, create_rgbd,
    filter_by_luminance, create_visualizer, apply_default_view, create_frustum,
)

import ArducamDepthCamera as ac  # noqa: E402

os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"

ALIGN_DIR = os.path.join(os.path.dirname(__file__), "..", "alignment", "calibration")


def load_calibration():
    import json
    with open(os.path.join(ALIGN_DIR, "rgb_intrinsics.json")) as f:
        rgb = json.load(f)
    with open(os.path.join(ALIGN_DIR, "tof_intrinsics.json")) as f:
        tof = json.load(f)
    with open(os.path.join(ALIGN_DIR, "extrinsics_rgb_tof.json")) as f:
        ext = json.load(f)

    K_rgb = np.array(rgb["camera_matrix"], dtype=np.float64)
    dist_rgb = np.array(rgb["distortion_coefficients"], dtype=np.float64)
    K_tof = np.array(tof["camera_matrix"], dtype=np.float64)
    dist_tof = np.array(tof["distortion_coefficients"], dtype=np.float64)
    R = np.array(ext["R"], dtype=np.float64)
    t = np.array(ext["t_mm"], dtype=np.float64)
    rgb_size = (rgb["image_size"]["width"], rgb["image_size"]["height"])
    tof_size = (tof["image_size"]["width"], tof["image_size"]["height"])
    return K_rgb, dist_rgb, K_tof, dist_tof, R, t, rgb_size, tof_size


def colorize_pointcloud(pcd, rgb_image, K, R, t):
    '''Assign per-point RGB colors by projecting points into the RGB camera.

    rgb_image must be pre-undistorted and K the corresponding (new) camera matrix,
    so a plain pinhole projection is valid.
    '''
    points = np.asarray(pcd.points)  # (N, 3), ToF camera frame
    h, w = rgb_image.shape[:2]

    # transform all points into the RGB camera frame
    pts_rgb = (R @ points.T).T + t  # (N, 3) in RGB camera frame

    valid = pts_rgb[:, 2] > 10.0  # in front of the camera (>= 10mm)
    x = pts_rgb[valid, 0] / pts_rgb[valid, 2]
    y = pts_rgb[valid, 1] / pts_rgb[valid, 2]
    # ideal pinhole projection (rgb_image is pre-undistorted, so K is the new_K)
    uv = np.stack([x * K[0, 0] + K[0, 2],
                   y * K[1, 1] + K[1, 2]], axis=1)

    inside = (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)

    colors = np.zeros((len(points), 3), dtype=np.float64)
    idx = np.where(valid)[0]
    uu = uv[inside, 0].astype(np.float32)
    vv = uv[inside, 1].astype(np.float32)
    # bilinear sampling
    x0 = np.floor(uu).astype(int)
    y0 = np.floor(vv).astype(int)
    x1 = np.clip(x0 + 1, 0, w - 1)
    y1 = np.clip(y0 + 1, 0, h - 1)
    x0 = np.clip(x0, 0, w - 1)
    y0 = np.clip(y0, 0, h - 1)
    dx = (uu - x0).reshape(-1, 1)
    dy = (vv - y0).reshape(-1, 1)
    top = rgb_image[y0, x0] * (1 - dx) + rgb_image[y0, x1] * dx
    bot = rgb_image[y1, x0] * (1 - dx) + rgb_image[y1, x1] * dx
    colors[idx[inside]] = (top * (1 - dy) + bot * dy)[:, ::-1] / 255.0

    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd


class RollingToFAverage:
    def __init__(self, frame_count):
        self.frames = deque(maxlen=frame_count)
        self.frame_count = frame_count
        self.depth_sum = None
        self.depth_count = None
        self.amplitude_sum = None

    def add(self, depth, amplitude):
        valid = np.isfinite(depth) & (depth > 0)
        depth_values = np.where(valid, depth, 0).astype(np.float64)
        amplitude_values = np.nan_to_num(amplitude, nan=0.0, posinf=0.0, neginf=0.0).astype(np.float64)
        if self.depth_sum is None:
            self.depth_sum = np.zeros_like(depth_values)
            self.depth_count = np.zeros_like(valid, dtype=np.int32)
            self.amplitude_sum = np.zeros_like(amplitude_values)
        if len(self.frames) == self.frame_count:
            old_depth, old_valid, old_amplitude = self.frames.popleft()
            self.depth_sum -= old_depth
            self.depth_count -= old_valid
            self.amplitude_sum -= old_amplitude
        self.frames.append((depth_values, valid, amplitude_values))
        self.depth_sum += depth_values
        self.depth_count += valid
        self.amplitude_sum += amplitude_values

    def mean(self):
        if len(self.frames) < self.frame_count:
            return None
        depth = np.divide(self.depth_sum, self.depth_count, out=np.zeros_like(self.depth_sum), where=self.depth_count > 0)
        return depth.astype(np.float32), (self.amplitude_sum / self.frame_count).astype(np.float32)


def read_tof_frame(cam, average):
    frame = cam.requestFrame(200)
    if frame is None or not isinstance(frame, ac.DepthData):
        return False
    try:
        average.add(frame.depth_data, frame.confidence_data)
    finally:
        cam.releaseFrame(frame)
    return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rgb-id", type=int, default=0)
    parser.add_argument("--tof-id", type=int, default=8)
    parser.add_argument("--confidence", type=int, default=20)
    parser.add_argument("--frames", type=int, default=8, help="rolling ToF frames to average (default: 20)")
    parser.add_argument("--save", default=os.path.join(os.path.dirname(__file__), "output", "pcd_colorized.ply"))
    args = parser.parse_args()
    if args.frames < 1:
        parser.error("--frames must be at least 1")

    K_rgb, dist_rgb, K_tof, dist_tof, R, t, rgb_size, tof_size = load_calibration()

    # --- stream RGB at the calibrated resolution ---
    picam2 = Picamera2(args.rgb_id)
    config = picam2.create_video_configuration({"size": rgb_size, "format": "RGB888"})
    picam2.configure(config)
    picam2.set_controls({"AfMode": 1, "AfSpeed": 1})
    rgb_path = os.path.join(os.path.dirname(__file__), "output", "colorize_rgb.jpg")
    rgb_lock = threading.Lock()
    stop_rgb = threading.Event()
    refocus_rgb = threading.Event()
    latest_rgb = [None, 0]
    rgb_error = [None]

    def rgb_worker():
        try:
            while not stop_rgb.is_set():
                if refocus_rgb.is_set():
                    refocus_rgb.clear()
                    if not picam2.autofocus_cycle(wait=True):
                        print("RGB autofocus failed; check the scene and press f to retry")
                frame = picam2.capture_array()  # RGB888 bytes are BGR for OpenCV
                with rgb_lock:
                    latest_rgb[0] = frame
                    latest_rgb[1] += 1
        except Exception as error:
            rgb_error[0] = error

    rgb_thread = None
    tof = None
    vis = None
    try:
        picam2.start()
        if not picam2.autofocus_cycle(wait=True):
            raise RuntimeError("RGB autofocus failed; check the scene and retry")
        new_K, _ = cv2.getOptimalNewCameraMatrix(K_rgb, dist_rgb, rgb_size, 0, rgb_size)
        rgb_thread = threading.Thread(target=rgb_worker, daemon=True)
        rgb_thread.start()

        tof = ac.ArducamCamera()
        ret = tof.open(ac.Connection.CSI, args.tof_id)
        if ret != 0:
            raise RuntimeError(f"Failed to open ToF camera: {ret}")
        if tof.start(ac.FrameType.DEPTH) != 0:
            raise RuntimeError("Failed to start ToF camera")
        driver_intrinsic = get_intrinsic_driver(tof)
        if (driver_intrinsic.width, driver_intrinsic.height) != tof_size:
            raise RuntimeError("ToF stream resolution differs from calibration")
        tof_new_K, _ = cv2.getOptimalNewCameraMatrix(
            K_tof, dist_tof, tof_size, 0, tof_size)
        tof_map_x, tof_map_y = cv2.initUndistortRectifyMap(
            K_tof, dist_tof, None, tof_new_K, tof_size, cv2.CV_32FC1)
        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            tof_size[0], tof_size[1], tof_new_K[0, 0], tof_new_K[1, 1],
            tof_new_K[0, 2], tof_new_K[1, 2])
        max_depth = tof.getControl(ac.Control.RANGE) or 4000
        average = RollingToFAverage(args.frames)

        cv2.namedWindow("depth", cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow("amplitude", cv2.WINDOW_AUTOSIZE)
        vis = create_visualizer(pointsize=1.5)
        vis.add_geometry(create_frustum())
        pcd = o3d.geometry.PointCloud()
        vis.add_geometry(pcd)
        apply_default_view(vis)
        quit_requested = [False]
        save_requested = [False]
        vis.register_key_callback(ord("q"), lambda _: quit_requested.__setitem__(0, True))
        vis.register_key_callback(ord("s"), lambda _: save_requested.__setitem__(0, True))
        vis.register_key_callback(ord("f"), lambda _: refocus_rgb.set())
        rgb_undistorted = None
        current_rgb = None
        rgb_version = 0
        missed = 0
        print("Warming up ToF; press s to save, f to refocus, q to quit")

        def save_cloud():
            if args.save and len(pcd.points):
                os.makedirs(os.path.dirname(args.save) or ".", exist_ok=True)
                os.makedirs(os.path.dirname(rgb_path), exist_ok=True)
                o3d.io.write_point_cloud(args.save, pcd, write_ascii=False)
                cv2.imwrite(rgb_path, current_rgb)
                print(f"Saved {len(pcd.points)} points to {args.save}")

        while not quit_requested[0]:
            if rgb_error[0] is not None:
                raise RuntimeError("RGB capture stopped") from rgb_error[0]
            if read_tof_frame(tof, average):
                missed = 0
            else:
                missed += 1
                if missed >= 10:
                    raise RuntimeError("ToF stream stopped providing frames")

            means = average.mean()
            with rgb_lock:
                frame, version = latest_rgb
            if means is not None and frame is not None:
                if version != rgb_version:
                    if (frame.shape[1], frame.shape[0]) != rgb_size:
                        raise RuntimeError("RGB stream resolution differs from calibration")
                    current_rgb = frame
                    rgb_undistorted = cv2.undistort(frame, K_rgb, dist_rgb, None, new_K)
                    rgb_version = version
                depth, amplitude = means
                depth = cv2.remap(depth, tof_map_x, tof_map_y, cv2.INTER_NEAREST)
                amplitude = cv2.remap(amplitude, tof_map_x, tof_map_y, cv2.INTER_NEAREST)
                zdepth = convert_distance_to_zdepth(depth, intrinsic)
                amp_clipped = np.clip(amplitude, 0, np.percentile(amplitude, 99))
                amp_norm = cv2.normalize(amp_clipped, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
                cv2.imshow("amplitude", amp_norm)
                depth_preview = np.clip(depth * (255.0 / max_depth), 0, 255).astype(np.uint8)
                cv2.imshow("depth", cv2.bitwise_not(depth_preview))
                rgbd_image = create_rgbd(amp_norm, zdepth)
                current_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
                current_pcd = filter_by_luminance(current_pcd, args.confidence)
                current_pcd = colorize_pointcloud(current_pcd, rgb_undistorted, new_K, R, t)
                pcd.points = current_pcd.points
                pcd.colors = current_pcd.colors
                vis.update_geometry(pcd)

            if not vis.poll_events():
                break
            vis.update_renderer()
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            if save_requested[0]:
                save_cloud()
                save_requested[0] = False

        save_cloud()
    finally:
        stop_rgb.set()
        if rgb_thread is not None:
            rgb_thread.join(timeout=2)
        if tof is not None:
            tof.close()
        picam2.close()
        if vis is not None:
            vis.destroy_window()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
