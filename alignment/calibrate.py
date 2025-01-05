'''
https://www.geeksforgeeks.org/camera-calibration-with-python-opencv/
https://docs.opencv.org/4.x/da/d0d/tutorial_camera_calibration_pattern.html
'''

import cv2 
import numpy as np 
import os 
import glob 
import json

def save_coefficients_as_json(mtx, dist, rvecs, tvecs, path):
    calibration_data = {
        'camera_matrix': mtx.tolist(),
        'distortion_coefficients': dist.tolist(),
        'rotation_vectors': [r_vec.flatten().tolist() for r_vec in rvecs],
        'translation_vectors': [t_vec.flatten().tolist() for t_vec in tvecs]
    }
    with open(path, 'w') as outfile:
        json.dump(calibration_data, outfile, indent=4)


input_dir = 'RGB/output' # 'alignment/images/distorted'
CHECKERBOARD = (6, 9) 
epsilon = 0.001
iterations = 30


# stop when specified accuracy or number of iterations is reached. 
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, iterations, epsilon) 

points_3D = [] 
points_2D = [] 

# 3D points real world coordinates 
objectp3d = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32) 
objectp3d[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) 
prev_img_shape = None

images = glob.glob(os.path.join(input_dir, '*.jpg')) 

for filename in images: 
    image = cv2.imread(filename) 
    grayColor = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) 

    # Find the chess board corners If desired number of corners are found in the image then ret = true 
    ret, corners = cv2.findChessboardCorners( grayColor, CHECKERBOARD, 
                    cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE) 

    # If desired number of corners can be detected then, refine the pixel coordinates and display them on the images of checker board 
    if ret == True: 
        points_3D.append(objectp3d) 

        # Refining pixel coordinates for given 2d points. 
        corners_refined = cv2.cornerSubPix(grayColor, corners, (11, 11), (-1, -1), criteria) 
        points_2D.append(corners_refined) 

        # Draw and display the corners 
        image = cv2.drawChessboardCorners(image, CHECKERBOARD, corners_refined, ret) 

    cv2.imshow('img', image) 
    cv2.waitKey(100) 
cv2.destroyAllWindows() 

# Perform camera calibration by passing points_3D and its corresponding pixel coordinates (points_2D) 
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(points_3D, points_2D, grayColor.shape[::-1], None, None) 

json_path = 'alignment/imx519_calibration.json'
save_coefficients_as_json(mtx, dist, rvecs, tvecs, json_path)

# print(" Camera matrix:", mtx) 
# print("\n Distortion coefficient:", dist) 
# print("\n Rotation Vectors:", rvecs) 
# print("\n Translation Vectors:", tvecs) 
