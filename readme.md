# Sensorpack

**! Work in progress !**

a Raspberry Pi Camera Array containing depth, thermal and RGB cameras.  
It requires Pi 5 for its two CSI ports. It also contains a Pi Pico to preprocess the thermal data, and might eventually receive a BNO055 absolute orientation sensor.

- Arducam ToF Camera ([info](https://www.arducam.com/time-of-flight-camera-raspberry-pi/))
- Arducam IMX519 16MP Autofocus  ([info](https://www.arducam.com/16mp-autofocus-camera-for-raspberry-pi/), [shop](https://www.arducam.com/product/imx519-autofocus-camera-module-for-raspberry-pi-arducam-b0371/))
- Pimoroni MLX90640 Thermal Camera Breakout 55° ([shop](https://shop.pimoroni.com/products/mlx90640-thermal-camera-breakout?variant=12536948654163))

This is a personal project. I want to gain some experience with the sensors and their calibration and registration.

## Enclosure

the enclosure is designed in 3ds Max and printed using Prusa Slicer. 

<img src="_STL\Screenshot.jpg" width="512"/>
<img src="_STL\case.jpg" width="512"/>


## ToF camera installation

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

**! far from finished !**

So far I just tried existing Pico implementations for 
- Circuitpython
- Micropython
- Pico SDK
- Arduino 

and modified them to send the image buffer via USB serial connection to a receiver script on the Pi.  

Andre Weinand's [Pico SDK implementation](https://github.com/weinand/thermal-imaging-camera) provides ~ 23 fps while Micropython is ~ 4 fps and Arduino & Circuitpython even worse.

## Libcamera and Autofocus

some Info about Libcamera commands:
- https://docs.arducam.com/Raspberry-Pi-Camera/Native-camera/Libcamera-User-Guide/#for-arducam-16mp64mp-autofocus-camera
- https://www.raspberrypi.com/documentation/computers/camera_software.html

example rpicam (= libcamera) command for autofocus, fixed exposure and gain for IMX519: 

    rpicam-still --autofocus-mode auto --autofocus-range normal --width 4656 --height 3496 --shutter 500000 --gain 2 -e png -o RGB/image.png

## Open3D 

visualization examples, used for point cloud:
https://www.open3d.org/html/python_example/visualization/index.html

because Raspberry Pi only supports OpenGL ES, which is not enough for Open3D, we need to switch to software rendering:

    import os
    os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"

## Joint Bilateral Upscaling

I plan to upscale Depth (240,180) and Thermal (32,24) data using Joint Bilateral Upscaling (JBU) with RGB as reference. It is forked from [Martin Zurowietz's](https://github.com/mzur) implementation of Johannes Kopf's [publication](https://johanneskopf.de/publications/jbu/).

the code is located in a separate repo:  
https://github.com/LaserBorg/pyJBU

I used Cython, multiprocessing and single channel kernels (instead of RGB) to significantly improve execution speed.