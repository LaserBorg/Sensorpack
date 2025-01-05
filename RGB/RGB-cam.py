from picamera2 import Picamera2, Preview
import datetime
import time
import os

class HDRCamera:
    def __init__(self, cam_id, fstops=1, output_dir="./"):
        self.fstops = fstops
        self.output_dir = output_dir

        self.picam2 = Picamera2(cam_id)
        self.picam2.start_preview(Preview.QTGL)

        self.preview_config = self.picam2.create_preview_configuration()
        self.image_config = self.picam2.create_still_configuration()

        self.ctrls_autofocus = {"AfMode": 1, "AfTrigger": 0, "AfSpeed": 1}  # AfModeEnum.Auto, AfSpeedEnum.Fast
        self.ctrls_fixedfocus = {"AfMode": 0}  # AfModeEnum.Manual

        self.picam2.configure(self.preview_config)
        self.picam2.set_controls(self.ctrls_autofocus)
        self.picam2.start()
        time.sleep(3)

        self.metadata = self.picam2.capture_metadata()
        self.get_aeb_params()

    def get_aeb_params(self):
        exp = int(self.metadata["ExposureTime"] * self.metadata["AnalogueGain"] * self.metadata["DigitalGain"])
        self.aeb_params = {"ColourGains": self.metadata["ColourGains"], 
                           "AEB": [{"exp": exp//2**self.fstops, "gain": 1},
                                   {"exp": exp//2,              "gain": 2},
                                   {"exp": exp,                 "gain": 2**self.fstops}]}

    def capture(self):
        # Lock the focus
        self.picam2.set_controls(self.ctrls_fixedfocus)

        # take AEB exposures
        if self.fstops > 0:
            for i in range(3):
                ctrls_aeb = {'AnalogueGain': self.aeb_params["AEB"][i]["gain"],
                            'ExposureTime': self.aeb_params["AEB"][i]["exp"], 
                            'ColourGains': self.aeb_params["ColourGains"]}
                self.picam2.set_controls(ctrls_aeb)
                time.sleep(0.5)

                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                output_path = os.path.join(self.output_dir, f"image{i}_{timestamp}.jpg")
                self.picam2.switch_mode_and_capture_file(self.image_config, output_path)
                time.sleep(1)

            # Switch back to continuous autofocus
            self.picam2.set_controls(self.ctrls_autofocus)
            #time.sleep(2)  # Wait for the autofocus to complete
        
        # take a single exposure
        else:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            output_path = os.path.join(self.output_dir, f"image_{timestamp}.jpg")
            self.picam2.switch_mode_and_capture_file(self.image_config, output_path)

    def close(self):
        self.picam2.close()


if __name__ == "__main__":
    output_dir = "RGB/output"
    os.makedirs(output_dir, exist_ok=True)
    
    hdrcamera = HDRCamera(0, fstops=0, output_dir=output_dir)
    hdrcamera.capture()
    hdrcamera.close()
