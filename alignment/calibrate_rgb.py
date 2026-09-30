'''
Calibrate RGB camera intrinsics (pinhole + distortion) from checkerboard images.

Expects images in alignment/img/rgb/ (from capture_calib.py) or any directory
of checkerboard photos.

Usage:
    python alignment/calibrate_rgb.py [--input alignment/img/rgb] [--squares 9x6] [--square-size 25]
'''

import argparse
import glob
import json
import os

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", default=os.path.join(os.path.dirname(__file__), "img", "rgb"))
    parser.add_argument("--squares", default="9x6", help="inner corners as WxH, e.g. 9x6")
    parser.add_argument("--square-size", type=float, default=25.0, help="square size in mm")
    parser.add_argument("--output", default=os.path.join(os.path.dirname(__file__), "calibration", "rgb_intrinsics.json"))
    args = parser.parse_args()

    board_w, board_h = (int(v) for v in args.squares.lower().split("x"))
    # 3D object points in the board frame (mm)
    objectp = np.zeros((board_w * board_h, 3), np.float32)
    objectp[:, :2] = np.mgrid[0:board_w, 0:board_h].T.reshape(-1, 2)
    objectp *= args.square_size

    images = sorted(glob.glob(os.path.join(args.input, "*.jpg")) + glob.glob(os.path.join(args.input, "*.png")))
    print(f"Found {len(images)} images in {args.input}")

    object_points = []
    image_points = []
    image_size = None
    failed = []

    for path in images:
        img = cv2.imread(path)
        if img is None:
            failed.append((path, "unreadable"))
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        size = (img.shape[1], img.shape[0])
        if image_size is not None and size != image_size:
            raise SystemExit(f"Mixed RGB resolutions: {path} is {size}, expected {image_size}")
        image_size = size

        ret, corners = cv2.findChessboardCornersSB(gray, (board_w, board_h))
        if not ret:
            failed.append((path, "no corners"))
            continue

        object_points.append(objectp)
        image_points.append(corners.reshape(-1, 1, 2))
        print(f"  ok   {os.path.basename(path)}")

    for path, reason in failed:
        print(f"  FAIL {os.path.basename(path)}: {reason}")

    if len(object_points) < 10:
        raise SystemExit(f"Not enough good images ({len(object_points)}). Need at least 10.")

    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None)

    # Root mean square pixel error over every detected corner.
    total_squared_error = 0.0
    total_corners = 0
    for i, (obj, img_pts) in enumerate(zip(object_points, image_points)):
        proj, _ = cv2.projectPoints(obj, rvecs[i], tvecs[i], K, dist)
        total_squared_error += np.sum((img_pts.reshape(-1, 2) - proj.reshape(-1, 2)) ** 2)
        total_corners += len(img_pts)
    rmse = np.sqrt(total_squared_error / total_corners)

    result = {
        "image_size": {"width": image_size[0], "height": image_size[1]},
        "board": {"squares": [board_w, board_h], "square_size_mm": args.square_size},
        "num_images": len(object_points),
        "reprojection_rmse_px": float(rmse),
        "camera_matrix": K.tolist(),
        "distortion_coefficients": dist.flatten().tolist(),
    }
    with open(args.output, "w") as f:
        json.dump(result, f, indent=4)

    print(f"\nReprojection RMSE: {rmse:.3f} px")
    print("Camera matrix:\n", np.round(K, 3))
    print("Distortion:", np.round(dist.flatten(), 5))
    print(f"Saved to {args.output}")


if __name__ == "__main__":
    main()
