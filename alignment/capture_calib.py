'''
Capture matched RGB + ToF checkerboard poses for calibration, with live previews.

Both cameras stream in background threads and are shown in OpenCV windows so you
can see exactly what you are shooting. Interact via the OpenCV window keyboard:

    c / Enter   capture the current pose
    f           re-trigger autofocus (do this when the board distance changes)
    q           quit

For each captured pose:
  - one RGB still  -> alignment/calib/rgb/pose_XX.jpg
  - one ToF frame  -> alignment/calib/tof/pose_XX_depth.png      (16-bit, mm)
                      alignment/calib/tof/pose_XX_amplitude.png  (8-bit, normalized)

The board must be fully visible in BOTH cameras for every pose. Hold the board
still while capturing. Capture 20-30 poses covering the whole image (corners,
edges, center) with tilt in all directions.

Usage:
    python alignment/capture_calib.py [--poses 25] [--rgb-id 0] [--tof-id 8]
'''

import argparse
import os
import threading
import time

import cv2
import numpy as np
from picamera2 import Picamera2

import ArducamDepthCamera as ac

# libcamera AfState values
AF_LOCKED = (2, 3)  # FocusedLocked, Focused


class State:
    '''Thread-safe shared state between camera threads and the display loop.'''
    def __init__(self):
        self.lock = threading.Lock()
        self.rgb = None          # latest full-res BGR frame
        self.tof_depth = None    # latest depth (float32, mm)
        self.tof_amp = None      # latest amplitude (float32)
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

        # frames flow immediately; AF lock is detected from metadata, not blocking
        # (capture_array returns RGB888 from the camera buffer; convert to BGR
        # for OpenCV display and for consistent saved JPEGs)
        frame = picam2.capture_array()
        if frame is not None:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            md = picam2.capture_metadata()
            with state.lock:
                state.rgb = frame
                if not state.af_locked and md.get("AfState", 0) in AF_LOCKED:
                    state.af_locked = True


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
        cam.releaseFrame(frame)
        with state.lock:
            state.tof_depth = depth
            state.tof_amp = amp


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
    parser.add_argument("--rgb-id", type=int, default=0)
    parser.add_argument("--tof-id", type=int, default=8)
    parser.add_argument("--outdir", default=os.path.join(os.path.dirname(__file__), "calib"))
    args = parser.parse_args()

    rgb_dir = os.path.join(args.outdir, "rgb")
    tof_dir = os.path.join(args.outdir, "tof")
    os.makedirs(rgb_dir, exist_ok=True)
    os.makedirs(tof_dir, exist_ok=True)

    # --- open RGB (video mode: fast preview stream; 1080p is plenty for calibration) ---
    picam2 = Picamera2(args.rgb_id)
    picam2.configure(picam2.create_video_configuration({"size": (1920, 1080)}))
    picam2.set_controls({"AfMode": 1, "AfSpeed": 1})  # auto, fast
    picam2.start()

    # --- open ToF ---
    tof = ac.ArducamCamera()
    ret = tof.open(ac.Connection.CSI, args.tof_id)
    if ret != 0:
        raise RuntimeError(f"Failed to open ToF camera: {ret}")
    tof.start(ac.FrameType.DEPTH)

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
                    depth = state.tof_depth
                    amp = state.tof_amp

                if rgb is None or depth is None or amp is None:
                    print("  waiting for frames...")
                    continue

                name = f"pose_{captured:02d}"
                rgb_path = os.path.join(rgb_dir, f"{name}.jpg")
                depth_path = os.path.join(tof_dir, f"{name}_depth.png")
                amp_path = os.path.join(tof_dir, f"{name}_amplitude.png")

                cv2.imwrite(rgb_path, rgb)
                cv2.imwrite(depth_path, (depth / 4000 * 65536).astype(np.uint16))
                cv2.imwrite(amp_path, normalize_amp(amp))

                print(f"  [{captured+1}/{args.poses}] saved {name} "
                      f"(focus {'locked' if af_locked else 'NOT locked - press f'})")
                captured += 1
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
