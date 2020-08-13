import numpy as np 
import open3d as o3d 
import trimesh 
import copy


def execute_global_registration(source_pcd, target_pcd, voxel_size,distance_threshold = None):
    
    def preprocess_point_cloud(pcd, voxel_size):
        # The FPFH feature is a 33-dimensional vector that describes the local geometric property of a point.
        pcd_down = pcd.voxel_down_sample(voxel_size)

        radius_normal = voxel_size * 3
        pcd_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=50))

        radius_feature = voxel_size * 5
        pcd_fpfh = o3d.registration.compute_fpfh_feature(pcd_down, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=20))
        return pcd_down, pcd_fpfh
    distance_threshold = voxel_size*1.5 if distance_threshold == None else distance_threshold
    source_down, source_fpfh = preprocess_point_cloud(source_pcd, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target_pcd, voxel_size)

    result = o3d.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
        o3d.registration.TransformationEstimationPointToPoint(True), 4, [
            o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.registration.RANSACConvergenceCriteria(4000000, 500))
    return result
def execute_local_registration(source_pcd,target_pcd,trans_init,threshold):
    source_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=20))
    target_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=20))

    reg_p2p = o3d.registration.registration_icp(source_pcd, target_pcd, threshold, trans_init,
            o3d.registration.TransformationEstimationPointToPlane(),
            o3d.registration.ICPConvergenceCriteria(max_iteration = 9000))
    return reg_p2p

def get_point_cloud(file):
    return o3d.io.read_point_cloud(file)

def compute_transformation_matrix(source_cloud,target_cloud):
    # computes the transformatio matrix that transforms the source cloud to match the target cloud
    # it will choose the matrix that minimizes the distance beween the two point clouds. 
    # See ICP algorithm.

    # create axes
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size = 1, origin = np.array([0,0,0]))

    ### compute global registration ##
    voxel_size = 0.01
    distance_threshold = voxel_size*10.0
    global_result = execute_global_registration(source_cloud,target_cloud,voxel_size,distance_threshold)

    ### compute local registration ###
    distance_threshold = voxel_size*9.0
    local_result = execute_local_registration(source_cloud,target_cloud,global_result.transformation, distance_threshold)

    return local_result.transformation

def extract_transform(matrix):

    decomposed = trimesh.transformations.decompose_matrix(matrix)

    for name, val in zip(decomposed ,["scale","shear","angles","translation","perspective"]):
        print("_______")
        print(name)
        print(val)

    return decomposed


def compute_best_transformation_matrix(pc_1_raw, pc_2_raw, N = 5, show_viz = True):
    # leverages the compute transformation matrix function above. But runs that function multiple times and extracts the best tranformation matrix
    
    # stores the scores for each iteration
    scores_dict = dict()

    # voxel_size down sampling
    voxel_size = 0.01
    pc_1_raw = pc_1_raw.voxel_down_sample(voxel_size)
    pc_2_raw = pc_2_raw.voxel_down_sample(voxel_size)
    
    # intial evaluation
    threshold = 0.05
    intial_evaluation = o3d.registration.evaluate_registration(pc_1_raw, pc_2_raw, threshold, np.eye(4))

    for i in range(N):

        print("reading point clouds. Iter #%s of %s ......" % (i+1, N))

        # copy point clouds
        pc_1 = copy.deepcopy(pc_1_raw)
        pc_2 = copy.deepcopy(pc_2_raw)

        # paint uniform color
        pc_1.paint_uniform_color([1,0,0]) # red
        pc_2.paint_uniform_color([0,0,1]) # blue

        # compute transform matrix and evaluate
        matrix = compute_transformation_matrix(pc_1,pc_2)
        evaluation = o3d.registration.evaluate_registration(pc_1, pc_2, threshold, matrix)
        
        # add to scores dict
        scores_dict[evaluation.fitness] = (matrix,evaluation)

    print("__________________________________________________________________________")
    print("###### Finished iterations ######")
    
    # read point clouds
    pc_1 = pc_1_raw
    pc_2 = pc_2_raw
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size = 1, origin = np.array([0,0,0]))

    # show scores
    print("scores: ",sorted(list(scores_dict.keys())))

    # extract best transform 
    best_score = max(scores_dict.keys())
    if best_score < 0.5: 
        print(" *** scores less than 0.5 are not very good. You may want to increase the interation number to get a better score. ***")
    best_transform, best_evaluation = scores_dict[best_score]
    print("best evaluation: \n", best_evaluation)
    print("best transform: \n", best_transform)

    # apply transform 
    pc_1.transform(best_transform)
    scale,shear,angles,translation,perspective = extract_transform(best_transform)
    Tx, Ty, Tz = translation
    Rx, Ry, Rz = angles
    ROS_input = [-Tx, -Ty, -Tz, Ry, Rx, Rz]
    print("\n____________________ \n copy this into ROS:")
    print(ROS_input)

    # plot final results
    if show_viz:
        o3d.visualization.draw_geometries([pc_2,pc_1,axes])

        pc_1.paint_uniform_color([1,0,0])
        pc_2.paint_uniform_color([0,0,1])

        o3d.visualization.draw_geometries([pc_2,pc_1,axes])

    return ROS_input, best_score

if __name__ == "__main__":
    cam_1_fn = "data/camera_1_c.ply"
    cam_2_fn = "data/camera_2_c.ply"

    pc_1_raw = o3d.io.read_point_cloud(cam_1_fn)
    pc_2_raw = o3d.io.read_point_cloud(cam_2_fn)

    compute_best_transformation_matrix(pc_1_raw,pc_2_raw, N = 1, show_viz = True)

