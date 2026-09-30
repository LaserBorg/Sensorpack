'''
Capture matched RGB + ToF checkerboard poses for calibration, with live previews.

Both cameras stream in background threads and are shown in OpenCV windows so you
can see exactly what you are shooting. Interact via the OpenCV window keyboard:

    c / Enter   capture the current pose
    f           re-trigger autofocus (do this when the board distance changes)
    q           quit

For each captured pose:
    - one RGB still  -> alignment/img/rgb/pose_XX.jpg
    - one ToF frame  -> alignment/img/tof/pose_XX_depth.png      (16-bit, mm)
                                            alignment/img/tof/pose_XX_amplitude.png  (8-bit, normalized)

The board must be fully visible in BOTH cameras for every pose. Hold the board
still while capturing. Capture 20-30 poses covering the whole image (corners,
edges, center) with tilt in all directions.

Usage:
    python alignment/capture_calib.py [--poses 25] [--rgb-id 0] [--tof-id 8]
'''

import argparse
import json
import os
import threading
import time

import cv2
import numpy as np
from picamera2 import Picamera2

import ArducamDepthCamera as ac

# libcamera AfState.Focused (3 means Failed).
AF_FOCUSED = 2
RGB_SIZE = (4056, 3040)


class State:
    '''Thread-safe shared state between camera threads and the display loop.'''
    def __init__(self):
        self.lock = threading.Lock()
        self.rgb = None          # latest full-res BGR frame
        self.rgb_time = None
        self.tof_depth = None    # latest depth (float32, mm)
        self.tof_amp = None      # latest amplitude (float32)
        self.tof_time = None
        self.af_request = False  # set by main thread to re-focus
        self.af_locked = False
        self.running = True


def rgb_worker(picam2, state):
    '''Stream RGB frames; trigger AF on request and track lock state from metadata.'''
    while state.running:
        with state.lock:
            need_focus = state.af_request
            state.af_request = False

        if need_focus:
            picam2.set_controls({"AfTrigger": 1})  # start autofocus
            with state.lock:
                state.af_locked = False

        request = picam2.capture_request(flush=True)
        try:
            frame = request.make_array("main")
            metadata = request.get_metadata()
            timestamp = time.monotonic()
        finally:
            request.release()
        with state.lock:
            state.rgb = frame  # RGB888 is BGR byte order for OpenCV
            state.rgb_time = timestamp
            state.af_locked = metadata.get("AfState") == AF_FOCUSED


def tof_worker(cam, state):
    '''Stream ToF depth + amplitude frames. Frames must be released so the
    driver's small cache can recycle, otherwise requestFrame blocks.'''
    while state.running:
        frame = cam.requestFrame(1000)
        if frame is None or not isinstance(frame, ac.DepthData):
            continue
        # copy out of the driver buffer before releasing
        depth = np.nan_to_num(frame.depth_data).copy()
        amp = frame.confidence_data.copy()
        timestamp = time.monotonic()
        cam.releaseFrame(frame)
        with state.lock:
            state.tof_depth = depth
            state.tof_amp = amp
            state.tof_time = timestamp


def normalize_amp(amp):
    clipped = np.clip(amp, 0, np.percentile(amp, 99))
    return cv2.normalize(clipped, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)


def resize_for_display(img, max_w=800):
    h, w = img.shape[:2]
    if w <= max_w:
        return img
    scale = max_w / w
    return cv2.resize(img, (max_w, int(h * scale)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--poses", type=int, default=25)
    parser.add_argument("--squares", default="9x6", help="inner corners as WxH")
    parser.add_argument("--rgb-id", type=int, default=0)
    parser.add_argument("--tof-id", type=int, default=8)
    parser.add_argument("--max-skew-ms", type=float, default=100, help="maximum RGB/ToF capture time difference")
    parser.add_argument("--outdir", default=os.path.join(os.path.dirname(__file__), "img"))
    args = parser.parse_args()
    if args.max_skew_ms <= 0:
        parser.error("--max-skew-ms must be positive")
    board_size = tuple(int(value) for value in args.squares.lower().split("x"))

    rgb_dir = os.path.join(args.outdir, "rgb")
    tof_dir = os.path.join(args.outdir, "tof")
    os.makedirs(rgb_dir, exist_ok=True)
    os.makedirs(tof_dir, exist_ok=True)

    # --- open RGB in the same 4:3 mode used by the colorizer ---
    picam2 = Picamera2(args.rgb_id)
    picam2.configure(picam2.create_video_configuration({"size": RGB_SIZE, "format": "RGB888"}))
    picam2.set_controls({"AfMode": 1, "AfSpeed": 1})  # auto, fast
    picam2.start()

    # --- open ToF ---
    tof = ac.ArducamCamera()
    ret = tof.open(ac.Connection.CSI, args.tof_id)
    if ret != 0:
        raise RuntimeError(f"Failed to open ToF camera: {ret}")
    tof.start(ac.FrameType.DEPTH)
    info = tof.getCameraInfo()
    intrinsics = {
        "source": "Arducam firmware controls (raw values divided by 100)",
        "image_size": {"width": info.width, "height": info.height},
        "camera_matrix": [
            [tof.getControl(ac.Control.INTRINSIC_FX) / 100.0, 0, tof.getControl(ac.Control.INTRINSIC_CX) / 100.0],
            [0, tof.getControl(ac.Control.INTRINSIC_FY) / 100.0, tof.getControl(ac.Control.INTRINSIC_CY) / 100.0],
            [0, 0, 1],
        ],
        "distortion_coefficients": [0, 0, 0, 0, 0],
    }
    intrinsics_path = os.path.join(os.path.dirname(__file__), "calibration", "tof_intrinsics.json")
    if os.path.exists(intrinsics_path):
        with open(intrinsics_path) as file:
            if json.load(file) != intrinsics:
                raise SystemExit(f"ToF intrinsics changed; use a new --outdir: {intrinsics_path}")
    else:
        with open(intrinsics_path, "w") as file:
            json.dump(intrinsics, file, indent=4)

    state = State()
    state.af_request = True  # focus on startup

    rgb_t = threading.Thread(target=rgb_worker, args=(picam2, state), daemon=True)
    tof_t = threading.Thread(target=tof_worker, args=(tof, state), daemon=True)
    rgb_t.start()
    tof_t.start()

    cv2.namedWindow("RGB", cv2.WINDOW_NORMAL)
    cv2.namedWindow("ToF (amplitude)", cv2.WINDOW_NORMAL)

    print("Controls: [c/Enter] capture pose  [f] re-focus  [q] quit")
    captured = 0
    next_pose = 0

    try:
        while captured < args.poses:
            with state.lock:
                rgb = state.rgb
                amp = state.tof_amp
                af_locked = state.af_locked

            if rgb is not None:
                cv2.imshow("RGB", resize_for_display(rgb))
            if amp is not None:
                cv2.imshow("ToF (amplitude)", resize_for_display(normalize_amp(amp)))

            key = cv2.waitKey(30) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("f"):
                with state.lock:
                    state.af_request = True
                    state.af_locked = False
                print("  re-focusing...")
            elif key in (ord("c"), 13, 10):  # c, Enter
                with state.lock:
                    rgb = state.rgb
                    rgb_time = state.rgb_time
                    depth = state.tof_depth
                    amp = state.tof_amp
                    tof_time = state.tof_time
                    af_locked = state.af_locked

                if rgb is None or depth is None or amp is None:
                    print("  waiting for frames...")
                    continue
                if not af_locked:
                    print("  not saved: RGB autofocus is not locked; press f and hold the board still")
                    continue
                now = time.monotonic()
                rgb_age_ms = (now - rgb_time) * 1000
                tof_age_ms = (now - tof_time) * 1000
                skew_ms = abs(rgb_time - tof_time) * 1000
                if max(rgb_age_ms, tof_age_ms) > 500 or skew_ms > args.max_skew_ms:
                    print(f"  not saved: frame age RGB={rgb_age_ms:.0f}ms ToF={tof_age_ms:.0f}ms, "
                          f"skew={skew_ms:.0f}ms (limit {args.max_skew_ms:.0f}ms)")
                    continue

                rgb_gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
                amp_preview = normalize_amp(amp)
                rgb_ok, _ = cv2.findChessboardCornersSB(rgb_gray, board_size)
                tof_ok, _ = cv2.findChessboardCornersSB(amp_preview, board_size)
                if not (rgb_ok and tof_ok):
                    print(f"  not saved: checkerboard corners RGB={rgb_ok}, ToF={tof_ok}; adjust board/lighting/distance")
                    continue

                name = f"pose_{next_pose:02d}"
                while any(os.path.exists(path) for path in (
                    os.path.join(rgb_dir, f"{name}.jpg"),
                    os.path.join(tof_dir, f"{name}_depth.png"),
                    os.path.join(tof_dir, f"{name}_amplitude.png"),
                )):
                    next_pose += 1
                    name = f"pose_{next_pose:02d}"
                rgb_path = os.path.join(rgb_dir, f"{name}.jpg")
                depth_path = os.path.join(tof_dir, f"{name}_depth.png")
                amp_path = os.path.join(tof_dir, f"{name}_amplitude.png")

                if not cv2.imwrite(rgb_path, rgb):
                    raise RuntimeError(f"Could not save {rgb_path}")
                if not cv2.imwrite(depth_path, np.clip(depth, 0, 65535).astype(np.uint16)):
                    raise RuntimeError(f"Could not save {depth_path}")
                if not cv2.imwrite(amp_path, amp_preview):
                    raise RuntimeError(f"Could not save {amp_path}")

                print(f"  [{captured+1}/{args.poses}] saved {name} "
                        f"(RGB/ToF skew {skew_ms:.0f}ms)")
                captured += 1
                next_pose += 1
    finally:
        state.running = False
        rgb_t.join(timeout=2)
        tof_t.join(timeout=2)
        picam2.close()
        tof.close()
        cv2.destroyAllWindows()

    print(f"\nDone. {captured} poses saved to {args.outdir}")


if __name__ == "__main__":
    main()
