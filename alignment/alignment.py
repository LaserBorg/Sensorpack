import cv2
import numpy as np
import os
import json


def align_img(img, position, rotation, scale, size, interpolate=False):

    def calculate_position(imgsize, size, position, scale):
        
        def translate_centered(imgsize, size, translate=(0, 0)):
            x = (size[0] - imgsize[0]) // 2 + translate[0]
            y = (size[1] - imgsize[1]) // 2 + translate[1]
            return x, y

        scaled_imgsize = (int(imgsize[0] * scale[0]), int(imgsize[1] * scale[1]))
        center_offset = np.array(position) - np.array(size) / 2
        x, y = translate_centered(scaled_imgsize, size, translate=center_offset.astype(int))
        return scaled_imgsize, x, y

    imgsize = (img.shape[1], img.shape[0])

    scaled_imgsize, x, y = calculate_position(imgsize, size, position, scale)

    interpolation = cv2.INTER_LINEAR if interpolate else cv2.INTER_NEAREST
    img = cv2.resize(img, scaled_imgsize, interpolation=interpolation)

    # Rotate the image
    center = (scaled_imgsize[0] // 2, scaled_imgsize[1] // 2)
    rotation_matrix = cv2.getRotationMatrix2D(center, rotation, 1.0)
    img_rotated = cv2.warpAffine(img, rotation_matrix, scaled_imgsize, flags=cv2.INTER_LINEAR)

    # resize canvas and place the rotated image centered with offset
    img_aligned = np.zeros((size[1], size[0], 3), dtype=np.uint8)
    img_aligned[y:y+scaled_imgsize[1], x:x+scaled_imgsize[0]] = img_rotated

    return img_aligned


if __name__ == '__main__':

    input_dir = "alignment/images/"
    output_dir = "alignment/images/aligned/"
    json_path = "alignment/alignment.json"

    # Load transforms from JSON
    with open(json_path, 'r') as file:
        transformations = json.load(file)

    canvas_size = np.array(transformations['Size'])


    # Load images
    depth = cv2.imread(os.path.join(input_dir, 'depth.png'))
    thermal = cv2.imread(os.path.join(input_dir,'thermal.png'))
    color = cv2.imread(os.path.join(input_dir, 'RGB.jpg'))
    color = cv2.resize(color, canvas_size)


    # Align
    d = transformations['Depth']
    depth_aligned = align_img(depth, np.array(d['position']), d['rotation'], np.array(d['scale']), canvas_size)
    
    t = transformations['Thermal']
    thermal_aligned = align_img(thermal, np.array(t['position']), t['rotation'], np.array(t['scale']), canvas_size)


    # Save and display
    cv2.imwrite(os.path.join(output_dir,'color_aligned.jpg'), color)
    cv2.imwrite(os.path.join(output_dir,'depth_aligned.jpg'), depth_aligned)
    cv2.imwrite(os.path.join(output_dir,'thermal_aligned.jpg'), thermal_aligned)

    cv2.imshow('Color', color)
    cv2.imshow('Depth aligned', depth_aligned)
    cv2.imshow('Thermal aligned', thermal_aligned)

    cv2.waitKey(0)
    cv2.destroyAllWindows()
