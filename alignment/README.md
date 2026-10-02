# Camera calibration

The active RGB/ToF workflow is in [the calibration plan](../plan.md). Work from
the repository root. `capture_calib.py` needs a Pi with both cameras and a GUI;
`calibrate_rgb.py` and `calibrate_extrinsics.py` run offline on captured images.

The printed target has **9x6 inner corners** (10x7 squares). Mount it flat and
measure the square pitch in millimeters. Keep the cameras rigidly mounted and
use the same measured `--square-size` for both calibrators:

```sh
python alignment/capture_calib.py --poses 1
# Check that the first saved RGB and ToF amplitude images show the board clearly.
python alignment/capture_calib.py --poses 24
python alignment/calibrate_rgb.py --square-size 25
python alignment/calibrate_tof.py --square-size 25
python alignment/calibrate_extrinsics.py --square-size 25
python ToF/colorize.py
```

The colorizer shows a live rolling 20-frame ToF average with the latest RGB
colors, alongside ToF depth and amplitude preview windows. In its Open3D
window, `s` saves the current cloud and RGB image, `f`
refocuses, and `q` saves the latest cloud and exits. Keep the scene stationary
over the averaging window; moving objects will blur or ghost.

Replace `25` with the measured pitch. Capture with `c`/Enter, refocus with `f`,
quit with `q`. A pose is saved only if both images contain detectable corners.
The second capture command uses the next unused pose number. Captures live in
the gitignored `img/`; the RGB and ToF intrinsic results and extrinsics are
written under `alignment/calibration/` for the colorizer to load.
Do not estimate extrinsics if the board cannot be seen in
the real ToF amplitude view. Check the reported RGB pixel RMSE and extrinsic
rotation/translation spread before trusting point colors.

For a new capture session, pass a different `--outdir` so old and new poses are
not mixed. Use that directory for `calibrate_rgb.py --input` and
`calibrate_extrinsics.py --calib-dir`. Keep the board steady for each paired shot;
the capture tool requires focused RGB and fresh RGB/ToF frames with at most
400 ms skew by default (`--max-skew-ms` adjusts the limit). Each frame must be
less than 500 ms old. A pose can still move during that interval, so a stand or
firm support is preferable to handholding. Pair latency affects extrinsics;
RGB-only reprojection RMSE reflects RGB image and corner quality, not timing
between cameras.
