# Tests

Run from the repository root with the project virtual environment:

```sh
/home/pi/venv/bin/python -m unittest discover -s test -p 'test_*.py' -v
```

`test_colorize.py` exercises the live colorizer without opening either camera
or an Open3D window. Fake frames check that the rolling ToF window ignores
invalid depth, evicts its oldest frame, and releases driver frames. A fake
viewer and RGB/ToF cameras check frustum-first initialization, repeated point
cloud updates, saving the latest result on exit, and closing both cameras. It does not verify
autofocus quality, frame rate, or rendering on the physical rig.

`test_rgb_hdr.py` uses a fake Picamera2 to check fixed-gain shutter bracketing,
locked auto controls, and per-shot metadata recording. It also merges synthetic
JPEG brackets into a floating-point `.hdr` file and checks response-curve reuse.
It does not measure the real camera's exposure or ISP response.