import cv2
import open3d as o3d
import numpy as np
import os

def get_intrinsic():
    width = 240
    height = 180

    fx = width / (2 * np.tan(0.5 * np.pi * 64.3 / 180))
    fy = height / (2 * np.tan(0.5 * np.pi * 50.4 / 180))
    cx = width / 2
    cy = height / 2

    camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
    return camera_intrinsic

def filter_points(pcd, confidence_threshold=20):
    colors = np.asarray(pcd.colors)
    green_channel = colors[:, 1]
    mask = green_channel >= (confidence_threshold / 255.0)
    pcd = pcd.select_by_index(np.where(mask)[0])
    return pcd

def process_depth(color, depth, camera_intrinsic):
    # denoise depth
    depth_denoised = cv2.bilateralFilter(depth, d=5, sigmaColor=75, sigmaSpace=75)

    # print stats
    print("depth_denoised", depth_denoised.shape, depth_denoised.dtype, (depth_denoised.min(), depth_denoised.max()))
    print("color", color.shape, color.dtype, (color.min(), color.max()))

    # convert color to RGB
    color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)

    # convert to open3d images
    depth_image = o3d.geometry.Image(depth_denoised)
    color_image = o3d.geometry.Image(color)

    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_image, depth_image, depth_scale=1.0, depth_trunc=4000.0, convert_rgb_to_intensity=False)

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, camera_intrinsic)

    # Filter points based on confidence
    filtered_pcd = filter_points(pcd, confidence_threshold=50)
    return filtered_pcd

def load_frame():
    color = cv2.imread('ToF/output/kitchen_amplitude.png', -1) 
    depth_img = cv2.imread('ToF/output/kitchen_int16.png', -1)
    depth = (depth_img / 32767 * 4000).astype(np.float32)
    return color, depth


os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"

camera_intrinsic = get_intrinsic()
color, depth = load_frame()
filtered_pcd = process_depth(color, depth, camera_intrinsic)

# Save the point cloud
o3d.io.write_point_cloud("ToF/output/pcd_rgbd.ply", filtered_pcd, write_ascii=True)


# visualize
vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(filtered_pcd)
render_option = vis.get_render_option()
render_option.point_size = 1.
render_option.background_color = [0, 0, 0]

view_control = vis.get_view_control()

vis.run()
vis.destroy_window()
