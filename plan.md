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
- Printed checkerboard: **9x6 inner corners** (10x7 squares). Mount it flat on a
  rigid backing. Measure the actual square pitch with calipers and pass that
  value in mm to both calibration commands; do not assume it is 25 mm.
- Keep RGB focus and capture resolution fixed after calibration. Use the same
  camera modes for calibration and colorization (2028x1520 RGB by default,
  configurable with `--rgb-size`; 240x180 ToF).
- First collect **one pilot pose** with the board fully in both views. Check
  whether its squares are distinguishable in the *real* ToF amplitude image.
  If detection fails, adjust distance, exposure/lighting and board angle before
  attempting 20-30 static paired poses spread across the shared field of view.
  Avoid motion while capturing and avoid extreme edge cropping.

## Phase 1: RGB intrinsics

Script: `alignment/calibrate_rgb.py`.

1. Capture 20-30 paired poses with `alignment/capture_calib.py` (RGB saved at
  2028x1520 by default). Save to `alignment/img/rgb/` and `alignment/img/tof/`.
2. `cv2.findChessboardCornersSB` on each RGB image; skip failed detections and
  reject mixed image sizes.
3. `cv2.calibrateCamera` → `K`, `dist`, per-view `rvecs`/`tvecs`.
4. Inspect reprojection RMSE over *all corners* (aim below ~0.5 px; check
  blurred or poorly distributed images if larger). Save `alignment/calibration/rgb_intrinsics.json`.

## Phase 2: ToF intrinsics

Script: `alignment/calibrate_tof.py`.

Calibrate from the captured ToF amplitude checkerboard images, using the same
board dimensions and measured square pitch as the RGB calibration. This writes
`alignment/calibration/tof_intrinsics.json`, including the detected image size
and board metadata. At least 10 useful views are required; use the reprojection
RMSE and corner distribution to assess the result.

1. **Distance test:** compare the median range of a flat central board patch
  with a tape measurement from near the ToF optical center at several distances;
  account for surface/lens offsets and ToF noise. Investigate a repeatable
  discrepancy before proceeding. Depth PNGs store *slant range in mm*.
2. **Corner test:** pilot-capture the ToF *amplitude* image, which is only
  240x180. The capture tool requires detection in both cameras and prints
  which view failed. A 9x6 board has only an **8x5 square pitch** between its
  outer inner corners, not the full 10x7 board width.
3. If real ToF amplitude does not resolve the grid reliably, **stop**: the
  implemented solvePnP path cannot infer ToF board corners from depth alone.
  A different detectable target or a validated depth-plane correspondence
  method would be needed before estimating extrinsics.

## Phase 3: RGB ↔ ToF extrinsics

Script: `alignment/calibrate_extrinsics.py`

1. Capture N (20-30) poses where the checkerboard is fully visible in **both**
   cameras. For each pose save a matched pair:
  - `alignment/img/rgb/pose_XX.jpg`
  - `alignment/img/tof/pose_XX_depth.png` + `pose_XX_amplitude.png`
   (Board static during the pair; no hardware sync needed.)
2. For each pose:
   - Detect corners in the RGB image → `solvePnP` → board pose in RGB frame
     (`T_rgb_board`).
  - Detect corners in the ToF amplitude image → `solvePnP`
     → board pose in ToF frame (`T_tof_board`).
3. Per-pose relative transform: `T_rgb_tof = T_rgb_board · inv(T_tof_board)`:
  `p_rgb = R · p_tof + t` (the direction used by the colorizer).
4. Average rotations with an SO(3) projection and translations with a mean;
  inspect translation and rotation spread, rejecting inconsistent board
  orientations. Re-capture poor or ambiguous poses rather than trusting an
  average over them.
5. Save to `alignment/calibration/extrinsics_rgb_tof.json` (R, t in mm, per-pose residuals).

## Phase 4: Colorized point cloud

Script: `ToF/colorize.py`

1. Autofocus RGB, then stream RGB and ToF together. Build a live point cloud
  from a rolling average of 20 ToF depth/amplitude frames (`--frames` sets
  the window size). Invalid or zero depth readings do not contribute to the
  mean. Keep the scene still over the averaging window to avoid ghosting.
2. For each point p (ToF camera coords):
   - `p_rgb = R · p + t` → undistort the RGB frame, project with its new K and
     bilinearly sample. Points outside the RGB image or behind the camera get
     no color. Calibrated RGB resolution must match the capture mode.
3. Assign per-point color, render/save PLY.
4. **Validation:** point cloud of the checkerboard should show a clean,
   undistorted grid; straight edges in the scene should stay straight.
  Press `s` in the Open3D window to save the current colored cloud and RGB
  frame, `f` to refocus, or `q` to save the latest cloud and exit.

## Phase 5 (later): Thermal

- Thermal intrinsics are hard at 32x24. Preferred: calibrate thermal ↔ RGB
  extrinsics from checkerboard poses (reuse RGB intrinsics for projection),
  or thermal ↔ ToF the same way as Phase 3.
- Then assign thermal values per point via nearest-neighbor lookup in the
  32x24 grid (blocky result is inherent to the sensor).

## Deliverables

| File | Content |
|---|---|
| `alignment/calibration/rgb_intrinsics.json` | K, dist, image size, reprojection error |
| `alignment/calibration/tof_intrinsics.json` | Firmware K, image size, distortion assumption |
| `alignment/calibration/extrinsics_rgb_tof.json` | R, t (mm), per-pose residuals |
| `alignment/calibrate_rgb.py` | Phase 1 script |
| `alignment/calibrate_extrinsics.py` | Phase 3 script |
| `ToF/colorize.py` | Phase 4 script |

## Before Capturing

1. Measure the printed squares; keep the board flat and the camera rig fixed.
2. Run `python alignment/capture_calib.py --poses 1`. Hold the board still and
  press `c` in the preview. A failed detection is not saved. Inspect the saved
  RGB, amplitude and depth files before collecting the remaining 20-30 poses.
3. Run `python alignment/capture_calib.py --poses 24` (it resumes at unused
  pose numbers), then `python alignment/calibrate_rgb.py --square-size MM` and
  `python alignment/calibrate_extrinsics.py --square-size MM` using the same
  measured pitch for both commands. Inspect RMSE and transform spread before
  `python ToF/colorize.py`. Never reuse extrinsics after moving either camera.
