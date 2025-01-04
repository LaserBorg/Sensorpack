# Sensorpack

**! Work in progress !**

a Raspberry Pi Camera Array containing depth, thermal and RGB cameras.  
The device contains a Pi Pico/Pico2 to preprocess the thermal sensor data. It might also receive a BNO055 absolute orientation sensor later.

This is a personal project. I want to gain some experience with the sensors and their calibration and registration.

I used a Pi5 as host since we need two CSI-ports, but you may also be successful with a CM4 or some CSI multiplexer.

## Hardware

<img src="3D-print\hardware.jpg" width="1600"/>

Devices:
- Arducam ToF Camera ([info](https://www.arducam.com/time-of-flight-camera-raspberry-pi/))
- Arducam IMX519 16MP Autofocus  ([info](https://www.arducam.com/16mp-autofocus-camera-for-raspberry-pi/), [shop](https://www.arducam.com/product/imx519-autofocus-camera-module-for-raspberry-pi-arducam-b0371/))
- Pimoroni MLX90640 Thermal Camera Breakout 55° ([shop](https://shop.pimoroni.com/products/mlx90640-thermal-camera-breakout?variant=12536948654163))
- Raspberry Pi Pico / Pico 2
- Raspberry Pi 5

The enclosure is designed in 3ds Max and printed using Prusa Slicer in PETG for durability. project (max), exports (obj) and slicer (3mf) files are included.

<img src="3D-print\Screenshot.jpg" width="512"/>

## clone including submodules

```
git clone https://github.com/LaserBorg/Sensorpack.git
cd Sensorpack  
git submodule update --init --recursive
```


## build Arducam Pivariety camera driver to install ToF and IMX519 

this was the original forum thread from April 2024 where I tried to get ToF + 16MP running with in Bookworm:  
https://forum.arducam.com/t/installation-tof-camera-fails-on-bookworm-could-not-open-device-node-dev-video0/5883/29

first, install Arducam Pivariety Driver for ToF as described ([troubleshooting](https://docs.arducam.com/Raspberry-Pi-Camera/Tof-camera/Troubleshooting/#4-cannot-be-used-on-raspberry-pi5)):
https://docs.arducam.com/Raspberry-Pi-Camera/Tof-camera/Getting-Started/

then install IMX519:    
https://docs.arducam.com/Raspberry-Pi-Camera/Native-camera/16MP-IMX519/

With IMX519 at CSI-0 and ToF at CSI-1 attached, make sure your /boot/firmware/config.txt looks like this:

    dtoverlay=imx519, cam0
    dtoverlay=arducam-pivariety

I used these commands to check if the cameras are connected: 

    dmesg | grep arducam
    media-ctl -p

my ToF camera is ID 8, which is required to initialize the camera driver.

## Thermal
I modified existing implementations to send the image buffer via USB serial connection to a receiver script on the host computer. 
The submodule repo contains forks for Pico implementations running:

- Circuitpython
- Micropython
- Pico SDK
- Arduino.

Currently I'm sticking with CircuitPython, simply for ease of use.

#### Performance
Andre Weinand's [Pico SDK implementation](https://github.com/weinand/thermal-imaging-camera) provides ~ 23 fps, but unfortunately I'm not fluent with C++ and Piko SDK.

Micropython was ~4 fps and Circuitpython is even worse, so I I swapped the Pico for a Pico 2, which improved the performance a bit.

## Libcamera and Autofocus

some Info about Libcamera commands:
- https://docs.arducam.com/Raspberry-Pi-Camera/Native-camera/Libcamera-User-Guide/#for-arducam-16mp64mp-autofocus-camera
- https://www.raspberrypi.com/documentation/computers/camera_software.html

example rpicam (= libcamera) command for autofocus, fixed exposure and gain for IMX519: 

    rpicam-still --autofocus-mode auto --autofocus-range normal --width 4656 --height 3496 --shutter 500000 --gain 2 -e png -o RGB/image.png

## Open3D 

<img src="ToF\tof.jpg" width="940"/>

#### point cloud rendering

I tried to replicate the Arducam pointcloud example (C++) using Python and used the Open3D [visualization examples](https://www.open3d.org/html/python_example/visualization/index.html) as a reference.

It works good so far, but I realized that the depth buffer contains raw distance readings, **which implies that its image plane is spherical, not planar**.  
Since (I think) Open3D doesn't support distortion coefficients, I tried using OpenCV to undistort the map (_cv2.fisheye.initUndistortRectifyMap_), but haven't calibrated the camera yet, so the necessary coefficients are unknown. **It'd be great if someone could support here.**

#### OpenGL
because Raspberry Pi only supports OpenGL ES which seems to be not compatible to Open3D, we need to switch to software rendering:

    import os
    os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"

## Joint Bilateral Upscaling

I plan to upscale Depth (240,180) and Thermal (32,24) data using Joint Bilateral Upscaling (JBU) with RGB as reference. It is forked from [Martin Zurowietz's](https://github.com/mzur) implementation of Johannes Kopf's [publication](https://johanneskopf.de/publications/jbu/).

the code is located in a separate repo:  
https://github.com/LaserBorg/pyJBU

I used Cython, multiprocessing and single channel kernels (instead of RGB) to significantly improve execution speed.

## BNO055 Absolute Orientation Sensor

I haven't integrated the sensor in the enclosure design yet, but a basic version of the sender code for CircuitPython (Pico) and the receiver script for the host (Pi5) is already provided for testing purposes.