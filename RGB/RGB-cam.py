import datetime
import json
import os
import time

import cv2
import numpy as np
from picamera2 import Picamera2, Preview

OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")


def merge_brackets(records, output_dir, stamp):
    images = [cv2.imread(os.path.join(output_dir, record["file"])) for record in records]
    if any(image is None for image in images):
        raise RuntimeError("Could not read all bracket JPEGs")
    if any(image.shape != images[0].shape for image in images):
        raise RuntimeError("Bracket images have different resolutions")
    gain = records[0]["analogue_gain"]
    digital_gain = records[0]["digital_gain"]
    colour_gains = np.array(records[0]["colour_gains"], dtype=np.float32)
    if any(abs(record["analogue_gain"] - gain) > 0.05 * gain or
           abs(record["digital_gain"] - digital_gain) > 0.05 * digital_gain or
           not np.allclose(record["colour_gains"], colour_gains, rtol=0.05)
           for record in records):
        raise RuntimeError("Gain or white balance changed across the bracket; cannot merge radiance")

    times = np.array([record["exposure_us"] for record in records], dtype=np.float32) * np.float32(1e-6)
    response_path = os.path.join(output_dir, "camera_response.npz")
    response = None
    if os.path.exists(response_path):
        with np.load(response_path, allow_pickle=False) as saved:
            if (saved["image_shape"].tolist() == list(images[0].shape) and
                    np.isclose(saved["analogue_gain"], gain, rtol=0.05) and
                    np.isclose(saved["digital_gain"], digital_gain, rtol=0.05) and
                    np.allclose(saved["colour_gains"], colour_gains, rtol=0.05)):
                response = saved["response"]
    if response is None:
        response = cv2.createCalibrateDebevec().process(images, times)
        np.savez(response_path, response=response, image_shape=images[0].shape,
                 analogue_gain=gain, digital_gain=digital_gain, colour_gains=colour_gains)

    hdr = cv2.createMergeDebevec().process(images, times, response)
    hdr_path = os.path.join(output_dir, f"hdr_{stamp}.hdr")
    if not cv2.imwrite(hdr_path, hdr):
        raise RuntimeError(f"Could not save HDR file: {hdr_path}")
    return hdr_path


class HDRCamera:
    def __init__(self, cam_id, fstops=2, output_dir="./"):
        self.fstops = fstops
        self.output_dir = output_dir

        self.picam2 = Picamera2(cam_id)
        self.picam2.start_preview(Preview.QTGL)

        self.preview_config = self.picam2.create_preview_configuration()
        self.picam2.configure(self.preview_config)
        self.picam2.set_controls({"AfMode": 1, "AfSpeed": 1})
        self.picam2.start()
        time.sleep(2)
        if not self.picam2.autofocus_cycle(wait=True):
            self.picam2.close()
            raise RuntimeError("RGB autofocus failed")

        self.metadata = self.picam2.capture_metadata()
        self.gain = float(self.metadata["AnalogueGain"])
        self.colour_gains = tuple(self.metadata["ColourGains"])
        self.lens_position = self.metadata.get("LensPosition")

    def get_aeb_params(self):
        shutter_us = int(self.metadata["ExposureTime"])
        if self.fstops <= 0:
            return [shutter_us]
        scale = 2 ** self.fstops
        return [max(1, round(shutter_us / scale)), shutter_us, round(shutter_us * scale)]

    def capture(self):
        os.makedirs(self.output_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        records = []
        for index, shutter_us in enumerate(self.get_aeb_params()):
            controls = {
                "AeEnable": False,
                "AwbEnable": False,
                "AnalogueGain": self.gain,
                "ExposureTime": shutter_us,
                "ColourGains": self.colour_gains,
                "AfMode": 0,
            }
            if self.lens_position is not None:
                controls["LensPosition"] = self.lens_position
            config = self.picam2.create_still_configuration(controls=controls)
            request = self.picam2.switch_mode_and_capture_request(config, delay=2)
            try:
                metadata = request.get_metadata()
                actual_shutter = int(metadata["ExposureTime"])
                actual_gain = float(metadata["AnalogueGain"])
                if abs(actual_shutter - shutter_us) > 0.05 * shutter_us or abs(actual_gain - self.gain) > 0.05 * self.gain:
                    raise RuntimeError(f"Exposure was not applied: requested {shutter_us} us at gain {self.gain}, got {actual_shutter} us at gain {actual_gain}")
                if not np.allclose(metadata["ColourGains"], self.colour_gains, rtol=0.05):
                    raise RuntimeError("White balance changed during the bracket")
                filename = f"image{index}_{stamp}.jpg"
                request.save("main", os.path.join(self.output_dir, filename))
                records.append({
                    "file": filename,
                    "requested_exposure_us": shutter_us,
                    "exposure_us": actual_shutter,
                    "analogue_gain": actual_gain,
                    "digital_gain": float(metadata.get("DigitalGain", 1.0)),
                    "colour_gains": list(metadata["ColourGains"]),
                    "lens_position": metadata.get("LensPosition"),
                })
            finally:
                request.release()
            with open(os.path.join(self.output_dir, f"bracket_{stamp}.json"), "w") as file:
                json.dump(records, file, indent=2)
        if len(records) > 1:
            print(f"Saved floating-point HDR to {merge_brackets(records, self.output_dir, stamp)}")
        return records

    def close(self):
        self.picam2.close()


if __name__ == "__main__":
    output_dir = OUTPUT_DIR
    os.makedirs(output_dir, exist_ok=True)
    
    hdrcamera = HDRCamera(0, fstops=2, output_dir=output_dir)
    try:
        hdrcamera.capture()
    finally:
        hdrcamera.close()
