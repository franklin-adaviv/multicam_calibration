import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d


# Configure depth and color streams...
# ...from Camera 1
pipeline_1 = rs.pipeline()
config_1 = rs.config()
config_1.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config_1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming from both cameras
pipeline_1.start(config_1)

try:
    while True:

        ### Camera 1 ###
        # Wait for a coherent pair of frames: depth and color
        frames_1 = pipeline_1.wait_for_frames()
        depth_frame_1 = frames_1.get_depth_frame()
        color_frame_1 = frames_1.get_color_frame()
        if not depth_frame_1 or not color_frame_1:
            continue
        # Convert images to numpy arrays
        depth_image_1 = np.asanyarray(depth_frame_1.get_data())
        color_image_1 = np.asanyarray(color_frame_1.get_data())
        # # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        # depth_colormap_1 = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_1, alpha=0.5), cv2.COLORMAP_JET)

        intrinsic.width, intrinsic.height, intrinsic.fx,
        intrinsic.fy, intrinsic.ppx, intrinsic.ppy

        pc_1 = rs.pointcloud()
        pc_1.map_to(color_frame_1)
        points = pc_1.calculate(depth_frame_1)
        

        # show in open 3d.
        points.export_to_ply("test.ply", color_frame_1)


finally:

    # Stop streaming
    pipeline_1.stop()
