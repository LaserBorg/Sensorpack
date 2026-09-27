'''
Build a ToF point cloud and assign per-point RGB colors using the calibrated
intrinsics + extrinsics.

Pipeline:
  1. capture one ToF frame (depth + amplitude) and one RGB still
  2. build the point cloud in the ToF camera frame (slant range -> z-depth)
  3. for each point: p_rgb = R @ p_tof + t  ->  project with RGB K/dist
  4. sample the RGB pixel (bilinear); points outside the image get no color

Usage:
    python ToF/colorize.py [--rgb-id 0] [--tof-id 8] [--save ToF/output/pcd_colorized.ply]
'''

import argparse
import os

import cv2
import numpy as np
import open3d as o3d
from picamera2 import Picamera2

import sys
sys.path.insert(0, os.path.dirname(__file__))
from lib.depth_utils import (  # noqa: E402
    get_intrinsic_driver, convert_distance_to_zdepth, create_rgbd,
    filter_by_luminance, create_visualizer, apply_default_view,
)

import ArducamDepthCamera as ac  # noqa: E402

os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"

ALIGN_DIR = os.path.join(os.path.dirname(__file__), "..", "alignment")


def load_calibration():
    import json
    with open(os.path.join(ALIGN_DIR, "rgb_intrinsics.json")) as f:
        rgb = json.load(f)
    with open(os.path.join(ALIGN_DIR, "extrinsics_rgb_tof.json")) as f:
        ext = json.load(f)

    K_rgb = np.array(rgb["camera_matrix"], dtype=np.float64)
    dist_rgb = np.array(rgb["distortion_coefficients"], dtype=np.float64)
    R = np.array(ext["R"], dtype=np.float64)
    t = np.array(ext["t_mm"], dtype=np.float64)
    return K_rgb, dist_rgb, R, t


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
    colors[idx[inside]] = (top * (1 - dy) + bot * dy) / 255.0

    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rgb-id", type=int, default=0)
    parser.add_argument("--tof-id", type=int, default=8)
    parser.add_argument("--confidence", type=int, default=20)
    parser.add_argument("--save", default=os.path.join(os.path.dirname(__file__), "output", "pcd_colorized.ply"))
    args = parser.parse_args()

    K_rgb, dist_rgb, R, t = load_calibration()

    # --- capture one RGB still (undistorted) ---
    picam2 = Picamera2(args.rgb_id)
    config = picam2.create_still_configuration()
    picam2.configure(config)
    picam2.start()
    import time
    time.sleep(2)
    rgb_path = os.path.join(os.path.dirname(__file__), "output", "colorize_rgb.jpg")
    picam2.capture_file(rgb_path)
    picam2.close()

    # undistort the RGB image so ideal pinhole projection is valid
    rgb_img = cv2.imread(rgb_path)
    h, w = rgb_img.shape[:2]
    new_K, _ = cv2.getOptimalNewCameraMatrix(K_rgb, dist_rgb, (w, h), 0, (w, h))
    rgb_undistorted = cv2.undistort(rgb_img, K_rgb, dist_rgb, None, new_K)
    # use new_K for projection
    K_proj = new_K

    # --- capture one ToF frame ---
    tof = ac.ArducamCamera()
    ret = tof.open(ac.Connection.CSI, args.tof_id)
    if ret != 0:
        raise RuntimeError(f"Failed to open ToF camera: {ret}")
    tof.start(ac.FrameType.DEPTH)
    intrinsic = get_intrinsic_driver(tof)

    frame = tof.requestFrame(2000)
    depth = np.nan_to_num(frame.depth_data)
    amplitude = frame.confidence_data
    tof.close()

    # --- build point cloud in ToF frame ---
    zdepth = convert_distance_to_zdepth(depth, intrinsic)
    amp_clipped = np.clip(amplitude, 0, np.percentile(amplitude, 99))
    amp_norm = cv2.normalize(amp_clipped, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    rgbd_image = create_rgbd(amp_norm, zdepth)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
    pcd = filter_by_luminance(pcd, args.confidence)

    # --- colorize ---
    pcd = colorize_pointcloud(pcd, rgb_undistorted, K_proj, R, t)

    if args.save:
        os.makedirs(os.path.dirname(args.save), exist_ok=True)
        o3d.io.write_point_cloud(args.save, pcd, write_ascii=False)
        print(f"Saved {len(pcd.points)} points to {args.save}")

    vis = create_visualizer()
    vis.add_geometry(pcd)
    apply_default_view(vis)
    vis.run()
    vis.destroy_window()


if __name__ == "__main__":
    main()
