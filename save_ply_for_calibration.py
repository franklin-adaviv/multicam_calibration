import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d
from calibrate import *

def save_dual_cam_ply(cam_1_fn, cam_2_fn, cam_1_id = '943222072534', cam_2_id = '912112072667' ):

    # Configure depth and color streams...
    # ...from Camera 1
    pipeline_1 = rs.pipeline()
    config_1 = rs.config()
    config_1.enable_device(cam_1_id)
    config_1.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config_1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # ...from Camera 2
    pipeline_2 = rs.pipeline()
    config_2 = rs.config()
    config_2.enable_device(cam_2_id)
    config_2.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config_2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming from both cameras
    pipeline_1.start(config_1)
    pipeline_2.start(config_2)

    # flag
    saved_ply = False

    try:
        while saved_ply == False:

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

            # get colored point cloud
            pc_1 = rs.pointcloud()
            pc_1.map_to(color_frame_1)
            pc_1_points = pc_1.calculate(depth_frame_1)
            
            # save points
            pc_1_points.export_to_ply(cam_1_fn, color_frame_1)


            ### Camera 2 ###
            # Wait for a coherent pair of frames: depth and color
            frames_2 = pipeline_2.wait_for_frames()
            depth_frame_2 = frames_2.get_depth_frame()
            color_frame_2 = frames_2.get_color_frame()
            if not depth_frame_2 or not color_frame_2:
                continue
            # Convert images to numpy arrays
            depth_image_2 = np.asanyarray(depth_frame_2.get_data())
            color_image_2 = np.asanyarray(color_frame_2.get_data())
            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap_2 = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_2, alpha=0.5), cv2.COLORMAP_JET)

            # get colored point cloud
            pc_2 = rs.pointcloud()
            pc_2.map_to(color_frame_2)
            pc_2_points = pc_2.calculate(depth_frame_2)
            
            # save points
            pc_2_points.export_to_ply(cam_2_fn, color_frame_2)

            saved_ply = True

    finally:

        # Stop streaming
        pipeline_1.stop()
        pipeline_2.stop()