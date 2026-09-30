import cv2
import numpy as np
import ArducamDepthCamera as ac
import open3d as o3d

from lib.depth_utils import save_frame, get_intrinsic_driver, create_frustum, convert_distance_to_zdepth, create_rgbd, filter_by_luminance, create_visualizer, apply_default_view

import os
os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"


def filter_buffer(buffer: np.ndarray, amplitude: np.ndarray) -> np.ndarray:
    buffer = np.nan_to_num(buffer)
    buffer[amplitude < amplitude_thres] = 0
    return buffer

def on_amplitude_changed(value):
    global amplitude_thres
    amplitude_thres = value


amplitude_thres = 20
colormap_amplitude = False

output_dir = "ToF/output/"
os.makedirs(output_dir, exist_ok=True)


def main(cam_id=0, frame_average=20, save_maps=False, max_depth=4000, confidence=20):
    cam = ac.ArducamCamera()

    bilateralfilter = False

    ret = 0
    ret = cam.open(ac.Connection.CSI, cam_id)
    if ret != 0:
        print("Failed to open camera. Error code:", ret)
        return

    ret = cam.start(ac.FrameType.DEPTH)
    if ret != 0:
        print("Failed to start camera. Error code:", ret)
        cam.close()
        return

    r = cam.getControl(ac.Control.RANGE)

    info = cam.getCameraInfo()
    # Use the firmware-calibrated intrinsics from the driver instead of assuming a nominal FOV
    camera_intrinsic = get_intrinsic_driver(cam)
    print(f"Camera resolution: {info.width}x{info.height}")
    print(f"Intrinsics (driver): fx={camera_intrinsic.intrinsic_matrix[0,0]:.2f} fy={camera_intrinsic.intrinsic_matrix[1,1]:.2f} cx={camera_intrinsic.intrinsic_matrix[0,2]:.2f} cy={camera_intrinsic.intrinsic_matrix[1,2]:.2f}")

    cv2.namedWindow("depth", cv2.WINDOW_AUTOSIZE)
    cv2.createTrackbar("amplitude", "depth", amplitude_thres, 255, on_amplitude_changed)

    # Open3D visualization
    vis = create_visualizer(pointsize=1.5)

    # Define the camera frustum
    frustum = create_frustum()
    vis.add_geometry(frustum)
    
    pcd = o3d.geometry.PointCloud()
    vis.add_geometry(pcd)

    # add_geometry() resets the view, so apply the default orientation afterwards
    apply_default_view(vis)

    exit_flag = False

    def exit_callback(vis):
        global exit_flag
        exit_flag = True

    vis.register_key_callback(ord("q"), exit_callback)

    depth_mean = None
    amplitude_mean = None
    frame_count = 0

    while not exit_flag:
        frame = cam.requestFrame(2000)
        if frame is not None and isinstance(frame, ac.DepthData):
            depth = np.nan_to_num(frame.depth_data)
            amplitude_buf = np.nan_to_num(frame.confidence_data)
            
            if frame_count == 0:
                depth_mean = depth.copy()
                amplitude_mean = amplitude_buf.copy()
            else:
                depth_mean = (depth_mean * frame_count + depth) / (frame_count + 1)
                amplitude_mean = (amplitude_mean * frame_count + amplitude_buf) / (frame_count + 1)
            frame_count += 1

            depth = filter_buffer(depth_mean.copy(), amplitude_mean)
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
            
            if bilateralfilter:
                depth = cv2.bilateralFilter(depth, d=5, sigmaColor=75, sigmaSpace=75)
            
            # create point cloud
            zdepth = convert_distance_to_zdepth(depth, camera_intrinsic)
            rgbd_image = create_rgbd(amplitude_normalized, zdepth)
            current_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, camera_intrinsic)

            # Filter points based on amplitude
            current_pcd = filter_by_luminance(current_pcd, confidence)
            
            pcd.points = current_pcd.points
            pcd.colors = current_pcd.colors

            # Reset after some iterations
            if frame_count >= frame_average:
                # save 16bit depth and 8bit amplitude frames
                if save_maps:
                    save_frame(amplitude_normalized, output_dir)
                    save_frame(depth, output_dir, is_depth=True, max_depth=max_depth)

                # save point cloud
                pcd_path = os.path.join(output_dir, "pcd.ply")
                o3d.io.write_point_cloud(pcd_path, pcd, write_ascii=False)

                depth_mean = None
                amplitude_mean = None
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
