'''
https://forums.raspberrypi.com/viewtopic.php?t=369522
'''

from picamera2 import Picamera2
import cv2

size = (1920, 1080)
center =((size[0]//2),(size[1]//2))

camera_mode = {
    "size": size,
    "format": 'XRGB8888',
    'preserve_ar': True
}

cam_id = 0

picam2 = Picamera2(cam_id)

configuration = picam2.create_still_configuration(main=camera_mode)  #  create_video_configuration
picam2.configure(configuration)
picam2.start()

while True:
    frame = picam2.capture_array()
    cv2.circle(frame, center, 10, (255, 0 , 255), -1)
    cv2.imshow('frame', frame)
    
    key = cv2.waitKey(1)
    if key == ord("q"):
        break
