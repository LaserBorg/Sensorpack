import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np


script = Path(__file__).resolve().parents[1] / "RGB" / "RGB-cam.py"
spec = importlib.util.spec_from_file_location("rgb_hdr", script)
rgb_hdr = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rgb_hdr)


class HDRTests(unittest.TestCase):
    def test_hdr_merge_reuses_response_at_same_settings(self):
        with tempfile.TemporaryDirectory() as output_dir:
            records = []
            for index, brightness in enumerate((40, 110, 220)):
                filename = f"image{index}.jpg"
                cv2.imwrite(str(Path(output_dir) / filename), np.full((32, 32, 3), brightness, np.uint8))
                records.append({"file": filename, "exposure_us": 2000 * 4 ** index,
                                "analogue_gain": 2.0, "digital_gain": 1.0,
                                "colour_gains": [1.2, 1.1]})

            hdr_path = rgb_hdr.merge_brackets(records, output_dir, "first")
            hdr = cv2.imread(hdr_path, cv2.IMREAD_UNCHANGED)
            self.assertEqual(hdr.dtype, np.float32)
            self.assertTrue(np.isfinite(hdr).all())
            self.assertTrue((Path(output_dir) / "camera_response.npz").exists())

            with patch.object(rgb_hdr.cv2, "createCalibrateDebevec", side_effect=AssertionError("recalibrated")):
                rgb_hdr.merge_brackets(records, output_dir, "second")

    def test_capture_locks_controls_and_records_measured_exposures(self):
        class Request:
            def __init__(self, controls):
                self.controls = controls
                self.released = False

            def get_metadata(self):
                return {"ExposureTime": self.controls["ExposureTime"],
                        "AnalogueGain": self.controls["AnalogueGain"],
                        "DigitalGain": 1.0, "ColourGains": self.controls["ColourGains"],
                        "LensPosition": self.controls["LensPosition"]}

            def save(self, stream, filename):
                Path(filename).write_bytes(b"test jpeg")

            def release(self):
                self.released = True

        class Camera:
            def __init__(self, cam_id):
                self.requests = []

            def start_preview(self, preview):
                pass

            def create_preview_configuration(self):
                return {}

            def configure(self, config):
                pass

            def set_controls(self, controls):
                pass

            def start(self):
                pass

            def autofocus_cycle(self, wait=True):
                return True

            def capture_metadata(self):
                return {"ExposureTime": 8000, "AnalogueGain": 2.0,
                        "ColourGains": (1.2, 1.1), "LensPosition": 1.5}

            def create_still_configuration(self, controls):
                return controls

            def switch_mode_and_capture_request(self, config, delay):
                request = Request(config)
                self.requests.append(request)
                return request

            def close(self):
                pass

        camera = Camera(0)
        with tempfile.TemporaryDirectory() as output_dir, \
                patch.object(rgb_hdr, "Picamera2", return_value=camera), \
                patch.object(rgb_hdr.time, "sleep"), \
                patch.object(rgb_hdr, "merge_brackets", return_value="test.hdr") as merge:
            hdr_camera = rgb_hdr.HDRCamera(0, fstops=2, output_dir=output_dir)
            try:
                records = hdr_camera.capture()
            finally:
                hdr_camera.close()
            self.assertEqual([record["exposure_us"] for record in records], [2000, 8000, 32000])
            self.assertEqual([request.controls["ExposureTime"] for request in camera.requests], [2000, 8000, 32000])
            self.assertTrue(all(request.controls["AnalogueGain"] == 2.0 and
                                request.controls["AeEnable"] is False and
                                request.controls["AwbEnable"] is False and
                                request.released for request in camera.requests))
            self.assertEqual(len(json.loads(next(Path(output_dir).glob("bracket_*.json")).read_text())), 3)
            merge.assert_called_once()


if __name__ == "__main__":
    unittest.main()