import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d
from calibrate import *



# Configure depth and color streams...
# ...from Camera 1
pipeline_1 = rs.pipeline()
config_1 = rs.config()
config_1.enable_device('943222072534')
config_1.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config_1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
# ...from Camera 2
pipeline_2 = rs.pipeline()
config_2 = rs.config()
config_2.enable_device('912112072667')
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
        pc_1_points.export_to_ply("camera_1_c.ply", color_frame_1)


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
        pc_2_points.export_to_ply("camera_2_c.ply", color_frame_2)

        saved_ply = True

finally:

    # Stop streaming
    pipeline_1.stop()
    pipeline_2.stop()

scores_dict = dict()

pc_1_raw = o3d.io.read_point_cloud("camera_1.ply")
pc_2_raw = o3d.io.read_point_cloud("camera_2.ply")

# # remove outliers
# pc_1_raw, ind = pc_1_raw.remove_radius_outlier(nb_points=16, radius=0.05)
# pc_2_raw, ind = pc_2_raw.remove_radius_outlier(nb_points=16, radius=0.05)

# down sample
voxel_size = 0.01
pc_1_raw = pc_1_raw.voxel_down_sample(voxel_size)
pc_2_raw = pc_2_raw.voxel_down_sample(voxel_size)

for i in range(2):
    print("reading point clouds. Iter # ", i)

    # copy poitn clouds
    pc_1 = copy.deepcopy(pc_1_raw)
    pc_2 = copy.deepcopy(pc_2_raw)

    # paint uniform color
    pc_1.paint_uniform_color([1,0,0]) # red
    pc_2.paint_uniform_color([0,0,1]) # blue

    # initial transformation and evaluation
    initial_displacement = np.array([0.0,0.0,-0.2])
    initial_rotation = np.array([0.0,0.0,np.pi/2])
    T = np.eye(4)
    # T[:3,:3] = pc_1.get_rotation_matrix_from_xyz(initial_rotation)
    # T[:3,3] = initial_displacement
    print("estimated transform: ")
    print(T)
    threshold = 0.05
    evaluation = o3d.registration.evaluate_registration(pc_1, pc_2, threshold, T)
    print(evaluation)
    print("Starting viz")

    # apply estimated transform
    pc_1.transform(T)


    # compute transform matrix and evaluate
    # o3d.visualization.draw_geometries([pc_1,pc_2,axes])
    matrix = compute_transformation_matrix(pc_1,pc_2)
    full_matrix = np.matmul(matrix,T)
    print("computed transform: ")
    print(full_matrix)
    evaluation = o3d.registration.evaluate_registration(pc_1, pc_2, threshold, matrix)
    print(evaluation)

    # transform 
    # pc_1 = pc_1.transform(matrix)

    # add to scores dict
    scores_dict[evaluation.fitness] = full_matrix
print("__________________________________________________________________________")
print("scores: ",scores_dict.keys())
pc_1 = o3d.io.read_point_cloud("camera_1.ply")
pc_2 = o3d.io.read_point_cloud("camera_2.ply")
axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size = 1, origin = np.array([0,0,0]))

# extract best transform 
best_score = max(scores_dict.keys())
print("best score: ", best_score)
best_transform = scores_dict[best_score]
print("best transform: ", best_transform)

# apply transform 
pc_1.transform(best_transform)
extract_transform(best_transform)

# plot final result
o3d.visualization.draw_geometries([pc_2,pc_1,axes])

pc_1.paint_uniform_color([1,0,0])
pc_2.paint_uniform_color([0,0,1])

o3d.visualization.draw_geometries([pc_2,pc_1,axes])


