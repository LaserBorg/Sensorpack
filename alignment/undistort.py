import cv2
import json
import numpy as np
import os
import glob

def undistort_image(filename, mtx, dist):
    image = cv2.imread(filename) 
    image_undistorted = cv2.undistort(image, mtx, dist)
    return image_undistorted


# Load the parameters from the JSON file
with open('alignment/calibration.json', 'r') as json_file:
    calibration_data = json.load(json_file)

mtx = np.array(calibration_data['camera_matrix'])
dist = np.array(calibration_data['distortion_coefficients'])
rvecs = [np.array(r_vec) for r_vec in calibration_data['rotation_vectors']]
tvecs = [np.array(t_vec).reshape(3, 1) for t_vec in calibration_data['translation_vectors']]


input_dir = 'alignment/images/distorted'
output_dir = 'alignment/images/undistorted'
os.makedirs(output_dir, exist_ok=True)

images = glob.glob(os.path.join(input_dir, '*.jpg')) 

for filename in images: 
    image_undistorted = undistort_image(filename, mtx, dist)

    # Save the undistorted image
    filename = os.path.splitext(os.path.basename(filename))[0]
    savepath = os.path.join(output_dir, filename + '.jpg')
    cv2.imwrite(savepath, image_undistorted)

    cv2.imshow('image_undistorted', image_undistorted)
    if cv2.waitKey(100) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
