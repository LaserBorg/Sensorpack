import os
import datetime
import subprocess

output_dir = "RGB/output"

for x in range(20):
    timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    output_path = os.path.join(output_dir, f"image_{timestamp}.jpg")

    command = f"rpicam-still --autofocus-mode auto --autofocus-range normal --width 4656 --height 3496 --gain 4 -o {output_path}"
    subprocess.run(command, shell=True)