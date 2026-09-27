# Calibration Plan

Goal: calibrate intrinsics and extrinsics of the RGB (Pi Camera Module 3) and ToF
(Arducam pivariety) cameras so that the ToF point cloud can be colorized with RGB
values per point. Thermal (MLX90640, 32x24) is deferred to a later phase.

## Conventions

- All transforms in OpenCV camera coordinates: x right, y down, z forward.
- Units: millimeters (ToF depth buffer is in mm).
- Both cameras use the pinhole model + distortion coefficients
  (`cv2.calibrateCamera` / `cv2.solvePnP`). No fisheye model needed (62° FOV).
- The ToF depth buffer is **slant range**; convert to z-depth with
  `convert_distance_to_zdepth()` before building point clouds.

## Prerequisites

- Cameras rigidly mounted in the final enclosure (extrinsics are only valid for
  this mechanical state — recalibrate after any hardware change).
- Checkerboard: 9x6 inner corners, ~25mm squares, high contrast, on a flat rigid
  board. Print at 100% scale and verify square size with calipers.
- ~20-30 board poses per calibration, covering the whole image (corners, edges,
  center), with tilt in all directions.

## Phase 1: RGB intrinsics

Script: `alignment/calibrate_rgb.py` (pattern: existing `alignment/calibrate.py`)

1. Capture 20-30 stills of the checkerboard with the RGB camera
   (`rpicam-still` or `RGB/RGB-cam.py`), fixed exposure/gain, full resolution.
   Save to `alignment/images/rgb/`.
2. `cv2.findChessboardCorners` + `cornerSubPix` on each image.
3. `cv2.calibrateCamera` → `K`, `dist`, per-view `rvecs`/`tvecs`.
4. Check mean reprojection error (target < 0.5 px).
5. Save to `alignment/rgb_intrinsics.json` (K, dist, image size, reprojection error).

## Phase 2: ToF intrinsics verification

The driver already provides firmware-calibrated intrinsics
(fx=190.92, fy=191.25, cx=120.0, cy=90.0 for 240x180), read via
`get_intrinsic_driver()`. Verify instead of re-calibrating:

1. **Distance test:** place the checkerboard at a known distance (e.g. 500mm,
   measured with a tape from the lens center), capture a ToF frame, build the
   point cloud, and measure the board's real-world size from the cloud
   (known: 8x5 squares = 200x125mm). Check scale error < 1-2%.
2. **Corner test (optional):** try `cv2.findChessboardCorners` on the ToF
   *amplitude* image (grayscale-like, board should be high-amplitude). If it
   works reliably, run `cv2.calibrateCamera` on the amplitude images and compare
   against the driver values. If not, keep the driver intrinsics.
3. Save the final values to `alignment/tof_intrinsics.json`.

## Phase 3: RGB ↔ ToF extrinsics

Script: `alignment/calibrate_extrinsics.py`

1. Capture N (20-30) poses where the checkerboard is fully visible in **both**
   cameras. For each pose save a matched pair:
   - `rgb/pose_XX.jpg`
   - `tof/pose_XX_depth.png` + `tof/pose_XX_amplitude.png`
   (Board static during the pair; no hardware sync needed.)
2. For each pose:
   - Detect corners in the RGB image → `solvePnP` → board pose in RGB frame
     (`T_rgb_board`).
   - Detect corners in the ToF amplitude image (fallback: segment the board
     plane from the point cloud and match corners geometrically) → `solvePnP`
     → board pose in ToF frame (`T_tof_board`).
3. Per-pose relative transform: `T_tof_rgb = T_tof_board · inv(T_rgb_board)`.
4. Average over all poses (e.g. procrustes / least-squares on rotation, mean
   translation) → final extrinsic.
5. Save to `alignment/extrinsics_rgb_tof.json` (R, t in mm, per-pose residuals).

## Phase 4: Colorized point cloud

Script: `ToF/colorize.py`

1. Load a ToF frame, build the point cloud (existing pipeline in `depth.py`).
2. For each point p (ToF camera coords):
   - `p_rgb = R · p + t` → project with RGB K/dist → sample RGB pixel
     (bilinear). Points outside the RGB image or behind the camera get no color.
3. Assign per-point color, render/save PLY.
4. **Validation:** point cloud of the checkerboard should show a clean,
   undistorted grid; straight edges in the scene should stay straight.

## Phase 5 (later): Thermal

- Thermal intrinsics are hard at 32x24. Preferred: calibrate thermal ↔ RGB
  extrinsics from checkerboard poses (reuse RGB intrinsics for projection),
  or thermal ↔ ToF the same way as Phase 3.
- Then assign thermal values per point via nearest-neighbor lookup in the
  32x24 grid (blocky result is inherent to the sensor).

## Deliverables

| File | Content |
|---|---|
| `alignment/rgb_intrinsics.json` | K, dist, image size, reprojection error |
| `alignment/tof_intrinsics.json` | K (driver or calibrated), image size |
| `alignment/extrinsics_rgb_tof.json` | R, t (mm), per-pose residuals |
| `alignment/calibrate_rgb.py` | Phase 1 script |
| `alignment/calibrate_extrinsics.py` | Phase 3 script |
| `ToF/colorize.py` | Phase 4 script |

## Open questions

- Does corner detection work on the ToF amplitude image? (decides Phase 2/3 path)
- Which point on the ToF lens is the optical center for distance measurements?
- Should extrinsics be anchored to the ToF frame (yes, since the point cloud
  lives there) — confirm no other frame is more convenient.
