'''
Calibrate ToF camera intrinsics from checkerboard amplitude images.

Expects images in alignment/img/tof/ (from capture_calib.py).

Usage:
    python alignment/calibrate_tof.py [--input alignment/img/tof] [--squares 9x6] [--square-size 25]
'''

import argparse
import glob
import json
import os

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", default=os.path.join(os.path.dirname(__file__), "img", "tof"))
    parser.add_argument("--squares", default="9x6", help="inner corners as WxH, e.g. 9x6")
    parser.add_argument("--square-size", type=float, default=25.0, help="square size in mm")
    parser.add_argument("--output", default=os.path.join(os.path.dirname(__file__), "calibration", "tof_intrinsics.json"))
    args = parser.parse_args()

    board_w, board_h = (int(value) for value in args.squares.lower().split("x"))
    objectp = np.zeros((board_w * board_h, 3), np.float32)
    objectp[:, :2] = np.mgrid[0:board_w, 0:board_h].T.reshape(-1, 2)
    objectp *= args.square_size

    images = sorted(glob.glob(os.path.join(args.input, "*_amplitude.png")))
    print(f"Found {len(images)} ToF amplitude images in {args.input}")

    object_points = []
    image_points = []
    image_size = None
    failed = []

    for path in images:
        gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            failed.append((path, "unreadable"))
            continue
        size = (gray.shape[1], gray.shape[0])
        if image_size is not None and size != image_size:
            raise SystemExit(f"Mixed ToF resolutions: {path} is {size}, expected {image_size}")
        image_size = size

        found, corners = cv2.findChessboardCornersSB(gray, (board_w, board_h))
        if not found:
            failed.append((path, "no corners"))
            continue

        object_points.append(objectp)
        image_points.append(corners.reshape(-1, 1, 2))
        print(f"  ok   {os.path.basename(path)}")

    for path, reason in failed:
        print(f"  FAIL {os.path.basename(path)}: {reason}")

    if len(object_points) < 10:
        raise SystemExit(f"Not enough good images ({len(object_points)}). Need at least 10.")

    _, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None)

    total_squared_error = 0.0
    total_corners = 0
    for index, (obj, img_pts) in enumerate(zip(object_points, image_points)):
        projected, _ = cv2.projectPoints(obj, rvecs[index], tvecs[index], K, dist)
        total_squared_error += np.sum((img_pts.reshape(-1, 2) - projected.reshape(-1, 2)) ** 2)
        total_corners += len(img_pts)
    rmse = np.sqrt(total_squared_error / total_corners)

    result = {
        "source": "Checkerboard calibration from ToF amplitude images",
        "image_size": {"width": image_size[0], "height": image_size[1]},
        "board": {"squares": [board_w, board_h], "square_size_mm": args.square_size},
        "num_images": len(object_points),
        "reprojection_rmse_px": float(rmse),
        "camera_matrix": K.tolist(),
        "distortion_coefficients": dist.flatten().tolist(),
    }
    os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
    with open(args.output, "w") as file:
        json.dump(result, file, indent=4)

    print(f"\nReprojection RMSE: {rmse:.3f} px")
    print("Camera matrix:\n", np.round(K, 3))
    print("Distortion:", np.round(dist.flatten(), 5))
    print(f"Saved to {args.output}")


if __name__ == "__main__":
    main()