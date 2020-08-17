import numpy as np
import pyrealsense2 as rs
import cv2
import open3d as o3d
from calibrate import *
import time

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

            # get colored point cloud
            pc_2 = rs.pointcloud()
            pc_2.map_to(color_frame_2)
            pc_2_points = pc_2.calculate(depth_frame_2)
            
            # save points
            pc_2_points.export_to_ply(cam_2_fn, color_frame_2)

            saved_ply = True
            print("sucessfully saved PLY")

    finally:

        # Stop streaming
        pipeline_1.stop()
        pipeline_2.stop()


if __name__ == "__main__":
    ### Get a some ply files for calibration ###
    N_samples = 2
    calibration_files = []

    print("starting the calibration procedure......")

    for i in range(N_samples):
        print("_________________________________________________")
        print("move the camera to the %s-th calibration position" % str(i+1))    

        print("[PRESS ENTER TO SAVE CALIBRATION DATA]")
        inp = raw_input()

        # do stuff
        cam_1_fn = "data/cam_1_calibration_%s.ply" % str(i+1)
        cam_2_fn = "data/cam_2_calibration_%s.ply" % str(i+1)
        save_dual_cam_ply(cam_1_fn,cam_2_fn, cam_1_id = '943222072534', cam_2_id = '912112072667')
        calibration_files.append([cam_1_fn,cam_2_fn])

        # done! 
        print("saved sample #%s of %s" %(i+1,N_samples))
    print("##########################################\n##########################################\nfinished data collection step for calibration")


    ### Run calibration ###

    print("starting transform computation step. For each sample, several iterations of the ICP algorithm will be run to find the best transformation matrix. The best of these best values will be used as the input into ROS. ")
    all_ros_inputs = []
    all_scores = []

    for ix in range(len(calibration_files)):

        cam_1_fn, cam_2_fn = calibration_files[ix]

        pc_1_raw = o3d.io.read_point_cloud(cam_1_fn)
        pc_2_raw = o3d.io.read_point_cloud(cam_2_fn)
        
        ROS_input, score = compute_best_transformation_matrix(pc_1_raw, pc_2_raw, N = 7, show_viz = False,show_prints = False)
        all_ros_inputs.append(ROS_input)
        all_scores.append(score)

        print("#####################################")
        print("        %s percent complete          "% str(int(100*(ix+1)/len(calibration_files))))
        print("#####################################")

    print("_________________________________________")
    print("all_scores: ")
    print(np.array(all_scores))
    print("all ROS inputs: ")
    print(np.array(all_ros_inputs))


    print("_________________________________________")
    print("final_score: ")
    ix = np.argmax(all_scores)
    print(all_scores[ix])
    print("final ROS input (copy this into the tf argument for base_to_camera1_tf in the launch file): ")
    print(np.array(all_ros_inputs)[ix,:])


