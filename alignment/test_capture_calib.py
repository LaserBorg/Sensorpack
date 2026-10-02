import unittest
from unittest.mock import patch

import numpy as np

from capture_calib import State, parse_rgb_size, rgb_worker


class Request:
    def __init__(self, focus_state):
        self.focus_state = focus_state
        self.released = False

    def make_array(self, stream):
        assert stream == "main"
        return np.array([[[0, 0, 255]]], dtype=np.uint8)

    def get_metadata(self):
        return {"AfState": self.focus_state}

    def release(self):
        self.released = True


class Camera:
    def __init__(self, state, focus_state):
        self.state = state
        self.request = Request(focus_state)
        self.flush = None
        self.calls = 0
        self.controls = []

    def set_controls(self, controls):
        self.controls.append(controls)

    def capture_request(self, flush=None):
        self.calls += 1
        self.flush = flush
        self.state.running = False
        return self.request


class CaptureTests(unittest.TestCase):
    def test_parse_configurable_rgb_size(self):
        self.assertEqual(parse_rgb_size("2028x1520"), (2028, 1520))

    def test_single_fresh_request_keeps_bgr_and_matching_focus_state(self):
        state = State()
        cam = Camera(state, 2)
        with patch("capture_calib.time.monotonic", return_value=123.0):
            rgb_worker(cam, state)

        self.assertEqual(cam.calls, 1)
        self.assertTrue(cam.flush)
        self.assertTrue(cam.request.released)
        np.testing.assert_array_equal(state.rgb[0, 0], [0, 0, 255])
        self.assertEqual(state.rgb_time, 123.0)
        self.assertTrue(state.af_locked)

    def test_failed_autofocus_is_not_treated_as_locked(self):
        state = State()
        cam = Camera(state, 3)
        rgb_worker(cam, state)

        self.assertFalse(state.af_locked)

    def test_focus_request_sends_libcamera_start_trigger(self):
        state = State()
        state.af_request = True
        cam = Camera(state, 2)

        rgb_worker(cam, state)

        self.assertEqual(cam.controls, [{"AfTrigger": 0}])


if __name__ == "__main__":
    unittest.main()