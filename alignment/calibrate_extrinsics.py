'''
Calibrate the RGB <-> ToF extrinsic (R, t) from matched checkerboard poses.

For each pose:
  - detect corners in the RGB image      -> solvePnP -> board pose in RGB frame
    - detect corners in the ToF amplitude   -> solvePnP -> board pose in ToF frame
    - relative transform: T_rgb_tof = T_rgb_board @ inv(T_tof_board)

The per-pose transforms are averaged (projected rotation mean, mean translation).

Expects matched pairs from capture_calib.py:
    alignment/img/rgb/pose_XX.jpg
    alignment/img/tof/pose_XX_amplitude.png

Usage:
    python alignment/calibrate_extrinsics.py [--squares 9x6] [--square-size 25]
'''

import argparse
import glob
import json
import os

import cv2
import numpy as np


def load_tof_intrinsics(json_path):
    '''Load ToF intrinsics and board metadata.'''
    with open(json_path) as file:
        data = json.load(file)
    K = np.array(data["camera_matrix"], dtype=np.float64)
    dist = np.array(data["distortion_coefficients"], dtype=np.float64)
    size = (data["image_size"]["width"], data["image_size"]["height"])
    return K, dist, size, data["board"]


def detect_corners(gray, board_size):
    ret, corners = cv2.findChessboardCornersSB(gray, board_size)
    return corners.reshape(-1, 1, 2) if ret else None


def pose_to_matrix(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.flatten()
    return T


def relative_transform(rgb_board, tof_board):
    return rgb_board @ np.linalg.inv(tof_board)


def average_transforms(transforms):
    '''Project the mean rotation onto SO(3) and average translations.'''
    left, _, right = np.linalg.svd(np.mean([T[:3, :3] for T in transforms], axis=0))
    correction = np.diag([1, 1, np.linalg.det(left @ right)])
    rotation = left @ correction @ right

    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = np.mean([T[:3, 3] for T in transforms], axis=0)
    return T


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--calib-dir", default=os.path.join(os.path.dirname(__file__), "img"))
    parser.add_argument("--rgb-intrinsics", default=os.path.join(os.path.dirname(__file__), "calibration", "rgb_intrinsics.json"))
    parser.add_argument("--tof-intrinsics", help="defaults to <calib-dir>/tof_intrinsics.json")
    parser.add_argument("--squares", default="9x6")
    parser.add_argument("--square-size", type=float, default=25.0)
    parser.add_argument("--output", default=os.path.join(os.path.dirname(__file__), "calibration", "extrinsics_rgb_tof.json"))
    args = parser.parse_args()

    board_w, board_h = (int(v) for v in args.squares.lower().split("x"))
    board_size = (board_w, board_h)
    objectp = np.zeros((board_w * board_h, 3), np.float32)
    objectp[:, :2] = np.mgrid[0:board_w, 0:board_h].T.reshape(-1, 2)
    objectp *= args.square_size

    tof_intrinsics_path = args.tof_intrinsics or os.path.join(os.path.dirname(__file__), "calibration", "tof_intrinsics.json")
    for path in (args.rgb_intrinsics, tof_intrinsics_path):
        if not os.path.isfile(path):
            raise SystemExit(f"Missing calibration: {path}. Run calibrate_rgb.py and calibrate_tof.py first.")

    # RGB intrinsics
    with open(args.rgb_intrinsics) as f:
        rgb_data = json.load(f)
    K_rgb = np.array(rgb_data["camera_matrix"], dtype=np.float64)
    dist_rgb = np.array(rgb_data["distortion_coefficients"], dtype=np.float64)
    rgb_size = (rgb_data["image_size"]["width"], rgb_data["image_size"]["height"])
    board = rgb_data["board"]
    if board["squares"] != [board_w, board_h] or board["square_size_mm"] != args.square_size:
        raise SystemExit("RGB intrinsics were calibrated with a different board; check --squares and --square-size")

    # ToF intrinsics
    K_tof, dist_tof, tof_size, tof_board = load_tof_intrinsics(tof_intrinsics_path)
    if tof_board["squares"] != [board_w, board_h] or tof_board["square_size_mm"] != args.square_size:
        raise SystemExit("ToF intrinsics were calibrated with a different board; check --squares and --square-size")
    print(f"ToF intrinsics: fx={K_tof[0,0]:.2f} fy={K_tof[1,1]:.2f} cx={K_tof[0,2]:.2f} cy={K_tof[1,2]:.2f} ({tof_size[0]}x{tof_size[1]})")

    rgb_files = sorted(glob.glob(os.path.join(args.calib_dir, "rgb", "pose_*.jpg")))
    print(f"Found {len(rgb_files)} RGB poses")

    transforms = []
    per_pose = []

    for rgb_path in rgb_files:
        name = os.path.basename(rgb_path)[:-4]  # pose_XX
        tof_amp_path = os.path.join(args.calib_dir, "tof", f"{name}_amplitude.png")
        if not os.path.exists(tof_amp_path):
            print(f"  SKIP {name}: missing ToF amplitude")
            continue

        rgb_img = cv2.imread(rgb_path)
        tof_amp = cv2.imread(tof_amp_path, cv2.IMREAD_GRAYSCALE)
        if rgb_img is None or tof_amp is None:
            print(f"  SKIP {name}: unreadable image")
            continue
        if (rgb_img.shape[1], rgb_img.shape[0]) != rgb_size or (tof_amp.shape[1], tof_amp.shape[0]) != tof_size:
            print(f"  SKIP {name}: image size differs from recorded intrinsics")
            continue

        corners_rgb = detect_corners(cv2.cvtColor(rgb_img, cv2.COLOR_BGR2GRAY), board_size)
        corners_tof = detect_corners(tof_amp, board_size)

        if corners_rgb is None:
            print(f"  SKIP {name}: no corners in RGB")
            continue
        if corners_tof is None:
            print(f"  SKIP {name}: no corners in ToF amplitude")
            continue

        ok_rgb, rvec_rgb, tvec_rgb = cv2.solvePnP(objectp, corners_rgb, K_rgb, dist_rgb, flags=cv2.SOLVEPNP_ITERATIVE)
        ok_tof, rvec_tof, tvec_tof = cv2.solvePnP(objectp, corners_tof, K_tof, dist_tof, flags=cv2.SOLVEPNP_ITERATIVE)

        if not (ok_rgb and ok_tof):
            print(f"  SKIP {name}: solvePnP failed")
            continue

        T_rgb_board = pose_to_matrix(rvec_rgb, tvec_rgb)
        T_tof_board = pose_to_matrix(rvec_tof, tvec_tof)
        T_rgb_tof = relative_transform(T_rgb_board, T_tof_board)

        transforms.append(T_rgb_tof)
        per_pose.append({
            "pose": name,
            "T_rgb_tof": T_rgb_tof.tolist(),
        })
        print(f"  ok   {name}")

    if len(transforms) < 10:
        raise SystemExit(f"Not enough good poses ({len(transforms)}). Need at least 10.")

    T_mean = average_transforms(transforms)

    # Reject a 180-degree board-order ambiguity before writing a calibration.
    angle_residuals = [float(np.degrees(np.arccos(np.clip(
        (np.trace(T[:3, :3] @ T_mean[:3, :3].T) - 1) / 2, -1, 1)))) for T in transforms]
    if max(angle_residuals) > 45:
        raise SystemExit("Inconsistent checkerboard orientation across views (rotation residual > 45 deg). Check paired poses and board orientation.")

    residuals = [float(np.linalg.norm(T[:3, 3] - T_mean[:3, 3])) for T in transforms]

    result = {
        "description": "Transform from ToF camera frame to RGB camera frame: p_rgb = R @ p_tof + t (t in mm)",
        "num_poses": len(transforms),
        "translation_residuals_mm": residuals,
        "rotation_residuals_deg": angle_residuals,
        "mean_residual_mm": float(np.mean(residuals)),
        "R": T_mean[:3, :3].tolist(),
        "t_mm": T_mean[:3, 3].tolist(),
        "per_pose": per_pose,
    }
    with open(args.output, "w") as f:
        json.dump(result, f, indent=4)

    print(f"\nUsed {len(transforms)} poses")
    print(f"Mean translation residual: {np.mean(residuals):.2f} mm (max {np.max(residuals):.2f})")
    print(f"Max rotation residual: {max(angle_residuals):.2f} deg")
    print("R:\n", np.round(T_mean[:3, :3], 5))
    print("t (mm):", np.round(T_mean[:3, 3], 2))
    print(f"Saved to {args.output}")


if __name__ == "__main__":
    main()
