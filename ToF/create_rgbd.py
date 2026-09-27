from lib.depth_utils import load_frame, get_intrinsic_driver, convert_distance_to_zdepth, create_rgbd, filter_by_luminance, create_visualizer, apply_default_view


if __name__ == "__main__":
    import cv2
    import open3d as o3d
    import os

    os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"

    print_stats = True
    save_pcd = True
    bilateralfilter = False
    
    color_path = "ToF/output/kitchen_amplitude.png"
    depth_path = "ToF/output/kitchen_int16.png"
    # color_path = "ToF/output/synthetic/amplitude.png"
    # depth_path = "ToF/output/synthetic/depth.png"

    max_depth = 4000
    confidence = 20

    color, depth = load_frame(color_path, depth_path, max_depth)

    if print_stats:
        print("color", color.shape, color.dtype, (color.min(), color.max()))
        print("depth", depth.shape, depth.dtype, (depth.min(), depth.max()))
        # cv2.imshow("color", color)
        # cv2.imshow("depth", depth)

    if bilateralfilter:
        depth = cv2.bilateralFilter(depth, d=5, sigmaColor=75, sigmaSpace=75)

    # Read the firmware-calibrated intrinsics from the driver (open camera for control values only)
    import ArducamDepthCamera as ac
    cam = ac.ArducamCamera()
    cam.open(ac.Connection.CSI, 8)
    camera_intrinsic = get_intrinsic_driver(cam)
    cam.close()
    print(camera_intrinsic.intrinsic_matrix)

    zdepth = convert_distance_to_zdepth(depth, camera_intrinsic)
    rgbd_image = create_rgbd(color, zdepth)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, camera_intrinsic)

    pcd = filter_by_luminance(pcd, confidence)

    if save_pcd:
        o3d.io.write_point_cloud("ToF/output/pcd_rgbd.ply", pcd, write_ascii=False)

    vis = create_visualizer()
    vis.add_geometry(pcd)
    apply_default_view(vis)  # add_geometry() resets the view
    vis.run()

    cv2.destroyAllWindows()
    vis.destroy_window()
