import open3d as o3d
import numpy as np
import cv2
import datetime
import os


def load_frame(color_path, depth_path, max_depth=4000):
    color = cv2.imread(color_path, -1)

    depth_img = cv2.imread(depth_path, -1)
    depth = (depth_img / 65536 * max_depth).astype(np.float32)

    return color, depth


def save_frame(frame, output_dir, is_depth=False, max_depth=4000):
    '''save as 16-bit PNG images'''
    timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")

    if is_depth:
        frame = frame / max_depth * 65536
        dtype = np.uint16
        filepath = os.path.join(output_dir, f"depth_{timestamp}.png")
    else: 
        dtype = np.uint8
        filepath = os.path.join(output_dir, f"amplitude_{timestamp}.png")
    
    cv2.imwrite(filepath, frame.astype(dtype))


def get_intrinsic(shape=(240,180), fov=70):

    def calculate_fov(shape, fov):
        aspect_ratio = shape[0] / shape[1]
        fov_rad = np.deg2rad(fov)

        hfov_rad = 2 * np.arctan(np.tan(fov_rad / 2) / np.sqrt(1 + (1 / aspect_ratio**2)))
        vfov_rad = 2 * np.arctan(np.tan(fov_rad / 2) / np.sqrt(1 + aspect_ratio**2))

        hfov = np.rad2deg(hfov_rad)
        vfov = np.rad2deg(vfov_rad)
        return [hfov, vfov]

    fovs = calculate_fov(shape, fov)
    width, height = shape

    fx = width / (2 * np.tan(0.5 * np.pi * fovs[0] / 180))
    fy = height / (2 * np.tan(0.5 * np.pi * fovs[1] / 180))
    cx = width / 2
    cy = height / 2

    # K = np.array([[fx, 0, cx],
    #               [0, fy, cy],
    #               [0, 0, 1]])

    camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
    return camera_intrinsic


def create_frustum(height=4000, fov=65, aspect_ratio=4/3):
    # frustum = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-2000, -1500, 0), max_bound=(2000, 1500, -4000))

    half_height = height
    half_width = half_height * np.tan(np.radians(fov / 2))
    half_depth = half_width / aspect_ratio

    vertices = [
        [0, 0, 0],  # Tip of the pyramid
        [-half_width, -half_depth, -half_height],  # Base vertices
        [half_width, -half_depth, -half_height],
        [half_width, half_depth, -half_height],
        [-half_width, half_depth, -half_height]
    ]

    lines = [
        [0, 1], [0, 2], [0, 3], [0, 4],  # Edges from the tip to the base
        [1, 2], [2, 3], [3, 4], [4, 1]   # Edges around the base
    ]

    colors = [[1, 0, 0] for _ in range(len(lines))]  # Red color for all lines

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(vertices)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set


def convert_distance_to_zdepth(depth, intrinsic):
    height, width = depth.shape
    fx = intrinsic.intrinsic_matrix[0, 0]
    fy = intrinsic.intrinsic_matrix[1, 1]
    cx = intrinsic.intrinsic_matrix[0, 2]
    cy = intrinsic.intrinsic_matrix[1, 2]

    x, y = np.meshgrid(np.arange(width), np.arange(height))
    x = (x - cx) / fx
    y = (y - cy) / fy

    zdepth = (depth / np.sqrt(x**2 + y**2 + 1)).astype(np.float32)
    return zdepth


def create_rgbd(color, depth):
    color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)

    depth_image = o3d.geometry.Image(depth)
    color_image = o3d.geometry.Image(color)

    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_image, depth_image, depth_scale=1.0, depth_trunc=4000.0, convert_rgb_to_intensity=False)

    return rgbd_image


def filter_by_luminance(pcd, confidence=20):
    green_channel = np.asarray(pcd.colors)[:, 1]
    mask = green_channel >= (confidence / 255.0)
    pcd = pcd.select_by_index(np.where(mask)[0])
    return pcd


def create_visualizer(shape=(640,480), pointsize=2., bgcolor=(0, 0, 0)):
    # vis = o3d.visualization.Visualizer()
    vis = o3d.visualization.VisualizerWithKeyCallback()

    vis.create_window("Point Cloud", shape[0], shape[1])
    render_option = vis.get_render_option()
    render_option.point_size = pointsize
    render_option.background_color = bgcolor

    view_control = vis.get_view_control()
    view_control.set_up([0, -1, 0])
    view_control.set_front([0, 0, -1])
    view_control.set_lookat([0, 0, 0])
    return vis
