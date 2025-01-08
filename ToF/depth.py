import cv2
import numpy as np
import ArducamDepthCamera as ac
import open3d as o3d
import datetime

import os
os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"


def filter_buffer(buffer: np.ndarray, amplitude: np.ndarray) -> np.ndarray:
    buffer = np.nan_to_num(buffer)
    buffer[amplitude < amplitude_thres] = 0
    return buffer

def filter_RGB_buffer(buffer: np.ndarray, amplitude: np.ndarray) -> np.ndarray:
    buffer = np.nan_to_num(buffer)
    buffer[amplitude < amplitude_thres] = (0, 0, 0)
    return buffer

def on_amplitude_changed(value):
    global amplitude_thres
    amplitude_thres = value

def filter_points(pcd, thres=20):
    colors = np.asarray(pcd.colors)
    green_channel = colors[:, 1]
    mask = green_channel >= (thres / 255.0)
    pcd = pcd.select_by_index(np.where(mask)[0])
    return pcd

def get_intrinsic(shape=(180, 240), fov=(64.3, 50.4)):
    height, width = shape

    # camera model parameters
    fx = width / (2 * np.tan(0.5 * np.pi * fov[0] / 180))
    fy = height / (2 * np.tan(0.5 * np.pi * fov[1] / 180))
    cx = width / 2
    cy = height / 2

    # K = np.array([[fx, 0, cx],
    #               [0, fy, cy],
    #               [0, 0, 1]])

    camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
    return camera_intrinsic

def process_depth(amplitude, depth, camera_intrinsic):
    amplitude = cv2.cvtColor(amplitude, cv2.COLOR_BGR2RGB)

    # Convert to Open3D images
    depth_image = o3d.geometry.Image(depth)
    color_image = o3d.geometry.Image(amplitude)

    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_image, depth_image, depth_scale=1.0, depth_trunc=4000.0, convert_rgb_to_intensity=False)

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, camera_intrinsic)

    # Rotate the point cloud 180 degrees around the x-axis
    R = pcd.get_rotation_matrix_from_xyz((np.pi, 0, 0))
    pcd.rotate(R, center=(0, 0, 0))

    return pcd

def save_frame(frame, output_dir, is_depth=False):
    '''save as 16-bit PNG images'''
    timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")

    if is_depth:
        frame = frame / 4000 * 32767
        dtype = np.uint16
        filepath = os.path.join(output_dir, f"depth_{timestamp}.png")
    else: 
        dtype = np.uint8
        filepath = os.path.join(output_dir, f"amplitude_{timestamp}.png")
    
    cv2.imwrite(filepath, frame.astype(dtype))

def create_frustum(height=4000, fov=65, aspect_ratio=4/3):
    # frustum = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-2000, -1500, 0), 
    #                                               max_bound=(2000, 1500, -4000))

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


amplitude_thres = 20
colormap_amplitude = False

output_dir = "ToF/output/"
os.makedirs(output_dir, exist_ok=True)


def main(cam_id=0, frame_average=0, save_maps=False):
    cam = ac.ArducamCamera()

    ret = 0
    ret = cam.open(ac.TOFConnect.CSI, cam_id)
    if ret != 0:
        print("Failed to open camera. Error code:", ret)
        return

    ret = cam.start(ac.TOFOutput.DEPTH)
    if ret != 0:
        print("Failed to start camera. Error code:", ret)
        cam.close()
        return

    r = cam.getControl(ac.TOFControl.RANGE)

    info = cam.getCameraInfo()
    camera_intrinsic = get_intrinsic(shape=(info.height, info.width))
    print(f"Camera resolution: {info.width}x{info.height}")

    cv2.namedWindow("depth", cv2.WINDOW_AUTOSIZE)
    cv2.createTrackbar("amplitude", "depth", amplitude_thres, 255, on_amplitude_changed)

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window("Point Cloud", 640, 480)
    vis.get_render_option().background_color = np.array([0, 0, 0])
    vis.get_render_option().point_size = 1.0

    # Define the camera frustum
    frustum = create_frustum()
    vis.add_geometry(frustum)
    
    pcd = o3d.geometry.PointCloud()
    vis.add_geometry(pcd)

    exit_flag = False

    def exit_callback(vis):
        global exit_flag
        exit_flag = True

    vis.register_key_callback(ord("q"), exit_callback)

    frame = cam.requestFrame(2000)
    depth_mean = frame.getDepthData()
    amplitude_mean = frame.getConfidenceData()
    frame_count = 0

    while not exit_flag:
        frame = cam.requestFrame(2000)
        if frame is not None and isinstance(frame, ac.DepthData):
            depth = frame.getDepthData()
            amplitude_buf = frame.getConfidenceData()
            
            depth_mean = (depth_mean * frame_count + depth) / (frame_count + 1)
            amplitude_mean = (amplitude_mean * frame_count + amplitude_buf) / (frame_count + 1)
            frame_count += 1

            depth = filter_buffer(depth_mean, amplitude_mean)
            amplitude_buf = amplitude_mean
            
            # normalized amplitude buffer
            amplitude_clipped = np.clip(amplitude_buf, 0, np.percentile(amplitude_buf, 99))
            amplitude_normalized = cv2.normalize(amplitude_clipped, None, 0, 1, cv2.NORM_MINMAX)
            amplitude_normalized = (amplitude_normalized * 255.0).astype(np.uint8)
            cv2.imshow("amplitude", amplitude_normalized)

            # colorized amplitude
            if colormap_amplitude:
                amplitude_normalized = cv2.applyColorMap(amplitude_normalized, cv2.COLORMAP_VIRIDIS)

            # colorized depth buffer
            colorized_depth = (depth * (255.0 / r)).astype(np.uint8)
            colorized_depth = cv2.bitwise_not(colorized_depth)
            colorized_depth = filter_buffer(colorized_depth, amplitude_buf)
            cv2.imshow("depth", colorized_depth)
            
            # denoise depth
            #depth = cv2.bilateralFilter(depth, d=5, sigmaColor=75, sigmaSpace=75)
            
            # create point cloud
            current_pcd = process_depth(amplitude_normalized, depth, camera_intrinsic)
            
            # Filter points based on amplitude
            #current_pcd = filter_points(current_pcd, thres=20)
            
            pcd.points = current_pcd.points
            pcd.colors = current_pcd.colors

            # Reset after some iterations
            if frame_count >= frame_average:
                # save 16bit depth and 8bit amplitude frames
                if save_maps:
                    save_frame(amplitude_normalized, output_dir)
                    save_frame(depth, output_dir, is_depth=True)

                # save point cloud
                pcd_path = os.path.join(output_dir, "pcd.ply")
                o3d.io.write_point_cloud(pcd_path, pcd, write_ascii=False)

                depth_mean = depth
                amplitude_mean = amplitude_buf
                frame_count = 0

            vis.update_geometry(pcd)
            vis.poll_events()
            vis.update_renderer()
            cam.releaseFrame(frame)

        key = cv2.waitKey(1)
        if key == ord("q"):
            break
    
    vis.destroy_window()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main(cam_id=8, frame_average=20, save_maps=False)
