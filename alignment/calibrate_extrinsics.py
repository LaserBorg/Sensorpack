'''
Calibrate the RGB <-> ToF extrinsic (R, t) from matched checkerboard poses.

For each pose:
  - detect corners in the RGB image      -> solvePnP -> board pose in RGB frame
  - detect corners in the ToF amplitude  -> solvePnP -> board pose in ToF frame
  - relative transform: T_tof_rgb = T_tof_board @ inv(T_rgb_board)

The per-pose transforms are averaged (quaternion mean for R, mean for t).

Expects matched pairs from capture_calib.py:
    alignment/calib/rgb/pose_XX.jpg
    alignment/calib/tof/pose_XX_amplitude.png

Usage:
    python alignment/calibrate_extrinsics.py [--squares 9x6] [--square-size 25]
'''

import argparse
import glob
import json
import os

import cv2
import numpy as np


def load_tof_intrinsics():
    '''Load ToF intrinsics from JSON if present, else query the driver live.'''
    json_path = os.path.join(os.path.dirname(__file__), "tof_intrinsics.json")
    if os.path.exists(json_path):
        with open(json_path) as f:
            data = json.load(f)
        K = np.array(data["camera_matrix"], dtype=np.float64)
        size = (data["image_size"]["width"], data["image_size"]["height"])
        return K, None, size

    import ArducamDepthCamera as ac
    cam = ac.ArducamCamera()
    ret = cam.open(ac.Connection.CSI, 8)
    if ret != 0:
        raise RuntimeError(f"Failed to open ToF camera: {ret}")
    K = np.array([
        [cam.getControl(ac.Control.INTRINSIC_FX) / 100.0, 0, cam.getControl(ac.Control.INTRINSIC_CX) / 100.0],
        [0, cam.getControl(ac.Control.INTRINSIC_FY) / 100.0, cam.getControl(ac.Control.INTRINSIC_CY) / 100.0],
        [0, 0, 1],
    ], dtype=np.float64)
    info = cam.getCameraInfo()
    cam.close()
    return K, None, (info.width, info.height)


def detect_corners(gray, board_size, criteria):
    ret, corners = cv2.findChessboardCorners(
        gray, board_size,
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ret:
        return None
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)


def pose_to_matrix(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.flatten()
    return T


def average_transforms(transforms):
    '''Average a list of 4x4 rigid transforms: quaternion mean for R, mean for t.'''
    def rot_to_quat(R):
        tr = np.trace(R)
        if tr > 0:
            s = np.sqrt(tr + 1.0) * 2
            q = np.array([s,
                          R[2, 1] - R[1, 2],
                          R[0, 2] - R[2, 0],
                          R[1, 0] - R[0, 1]]) / s
        else:
            q = np.zeros(4)
            ii = int(np.argmax(np.diag(R)))
            if ii == 0:
                s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
                q[0] = s
                q[1:] = (R[0, 1] + R[1, 0], R[0, 2] + R[2, 0], R[1, 0] - R[0, 1]) / s
            elif ii == 1:
                s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
                q[1] = s
                q[0] = (R[0, 1] + R[1, 0]) / s
                q[2:] = (R[1, 2] + R[2, 1], R[2, 1] - R[1, 2]) / s
            else:
                s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
                q[2] = s
                q[0] = (R[0, 2] + R[2, 0]) / s
                q[1] = (R[1, 2] + R[2, 1]) / s
                q[3] = (R[2, 0] - R[0, 2]) / s
        return q / np.linalg.norm(q)

    quats = np.array([rot_to_quat(T[:3, :3]) for T in transforms])
    # make all quaternions in the same hemisphere (q and -q are the same rotation)
    ref = quats[0]
    quats = np.where((quats @ ref < 0)[:, None], -quats, quats)
    q_mean = quats.mean(axis=0)
    q_mean /= np.linalg.norm(q_mean)

    # quaternion (w,x,y,z) -> rotation matrix
    w, x, y, z = q_mean
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])

    t_mean = np.mean([T[:3, 3] for T in transforms], axis=0)

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t_mean
    return T


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--calib-dir", default=os.path.join(os.path.dirname(__file__), "calib"))
    parser.add_argument("--rgb-intrinsics", default=os.path.join(os.path.dirname(__file__), "rgb_intrinsics.json"))
    parser.add_argument("--squares", default="9x6")
    parser.add_argument("--square-size", type=float, default=25.0)
    parser.add_argument("--output", default=os.path.join(os.path.dirname(__file__), "extrinsics_rgb_tof.json"))
    args = parser.parse_args()

    board_w, board_h = (int(v) for v in args.squares.lower().split("x"))
    board_size = (board_w, board_h)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    objectp = np.zeros((board_w * board_h, 3), np.float32)
    objectp[:, :2] = np.mgrid[0:board_w, 0:board_h].T.reshape(-1, 2)
    objectp *= args.square_size

    # RGB intrinsics
    with open(args.rgb_intrinsics) as f:
        rgb_data = json.load(f)
    K_rgb = np.array(rgb_data["camera_matrix"], dtype=np.float64)
    dist_rgb = np.array(rgb_data["distortion_coefficients"], dtype=np.float64)

    # ToF intrinsics
    K_tof, dist_tof, tof_size = load_tof_intrinsics()
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

        corners_rgb = detect_corners(cv2.cvtColor(rgb_img, cv2.COLOR_BGR2GRAY), board_size, criteria)
        corners_tof = detect_corners(tof_amp, board_size, criteria)

        if corners_rgb is None:
            print(f"  SKIP {name}: no corners in RGB")
            continue
        if corners_tof is None:
            print(f"  SKIP {name}: no corners in ToF amplitude")
            continue

        ok_rgb, rvec_rgb, tvec_rgb, _ = cv2.solvePnP(objectp, corners_rgb, K_rgb, dist_rgb, flags=cv2.SOLVEPNP_ITERATIVE)
        ok_tof, rvec_tof, tvec_tof, _ = cv2.solvePnP(objectp, corners_tof, K_tof, dist_tof, flags=cv2.SOLVEPNP_ITERATIVE)

        if not (ok_rgb and ok_tof):
            print(f"  SKIP {name}: solvePnP failed")
            continue

        T_rgb_board = pose_to_matrix(rvec_rgb, tvec_rgb)
        T_tof_board = pose_to_matrix(rvec_tof, tvec_tof)
        T_tof_rgb = T_tof_board @ np.linalg.inv(T_rgb_board)

        transforms.append(T_tof_rgb)
        per_pose.append({
            "pose": name,
            "T_tof_rgb": T_tof_rgb.tolist(),
        })
        print(f"  ok   {name}")

    if len(transforms) < 10:
        raise SystemExit(f"Not enough good poses ({len(transforms)}). Need at least 10.")

    T_mean = average_transforms(transforms)

    # per-pose residuals: distance of each pose's translation from the mean
    residuals = [float(np.linalg.norm(T[:3, 3] - T_mean[:3, 3])) for T in transforms]

    result = {
        "description": "Transform from RGB camera frame to ToF camera frame: p_tof = R @ p_rgb + t (t in mm)",
        "num_poses": len(transforms),
        "translation_residuals_mm": residuals,
        "mean_residual_mm": float(np.mean(residuals)),
        "R": T_mean[:3, :3].tolist(),
        "t_mm": T_mean[:3, 3].tolist(),
        "per_pose": per_pose,
    }
    with open(args.output, "w") as f:
        json.dump(result, f, indent=4)

    print(f"\nUsed {len(transforms)} poses")
    print(f"Mean translation residual: {np.mean(residuals):.2f} mm (max {np.max(residuals):.2f})")
    print("R:\n", np.round(T_mean[:3, :3], 5))
    print("t (mm):", np.round(T_mean[:3, 3], 2))
    print(f"Saved to {args.output}")


if __name__ == "__main__":
    main()
