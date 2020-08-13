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

	# # initail vizualization
	# o3d.visualization.draw_geometries([source_cloud,target_cloud,axes])

	#### visuzalize manual rotation ###
	# initial_displacement = -np.array([-0.1325,-0.1975,0.0])
	# initial_rotation = np.array([-1.570796327,0.0,0])
	# R = target_cloud.get_rotation_matrix_from_zyx(initial_rotation)
	# target_cloud_a = copy.deepcopy(target_cloud)
	# target_cloud_a.paint_uniform_color([0,0,1])66666
	# target_cloud.translate(initial_displacement)
	# target_cloud.rotate(R, center=(0, 0, 0))
	# o3d.visualization.draw_geometries([target_cloud_a,source_cloud,target_cloud,axes])

	### compute global registration ##
	voxel_size = 0.01
	distance_threshold = voxel_size*10.0
	global_result = execute_global_registration(source_cloud,target_cloud,voxel_size,distance_threshold)
	print("global: ")
	print(global_result.transformation)

	### compute local registration ###
	distance_threshold = voxel_size*9.0
	local_result = execute_local_registration(source_cloud,target_cloud,global_result.transformation, distance_threshold)
	print("local: ")
	print(local_result.transformation)

	# change colors of point clouds
	# source_cloud.paint_uniform_color([0,1,1]) # teal
	# target_cloud.paint_uniform_color([1,0,0]) # red

	# transform 
	transformed_source_cloud = copy.deepcopy(source_cloud)
	transformed_source_cloud.transform(local_result.transformation)

	# plot
	# axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size = 1, origin = np.array([0,0,0]))
	# o3d.visualization.draw_geometries([transformed_source_cloud,target_cloud,axes])

	return local_result.transformation

def extract_transform(matrix):

	for name, val in zip(trimesh.transformations.decompose_matrix(matrix) ,["scale","shear","angles","translation","perspective"]):
		print("_______")
		print(name)
		print(val)

if __name__ == "__main__":
	# read data 
	target = get_point_cloud("Testing_multicam_fixed.pcd")#("Testing_multicam_1597076460155095.pcd")#("f1.pcd") # top camera
	source = get_point_cloud("Testing_multicam_rot_5.pcd")#("Testing_multicam_1597076391518815.pcd")#"f2.pcd") # bottom Camera
	
	axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size = 1, origin = np.array([0,0,0]))
	o3d.visualization.draw_geometries([source,axes])
	# target.paint_uniform_color([1,0,0])
	# source.paint_uniform_color([0,1,0])
	### flip z ##
	# source_pts = np.asarray(source.points)[:,2]

	### preprocessing positions ###
	# initial_rotation = np.array([0.0,0.0,np.pi])
	# R = source.get_rotation_matrix_from_xyz(initial_rotation)	
	# source.rotate(R,center = (0,0,0))
	
	# get transformation matrix
	matrix = compute_transformation_matrix(source,target)
	extract_transform(matrix)


	A = np.array([[-0.99341053, -0.06755151,  0.09258674, -0.02571362],
				  [ 0.07032668, -0.99715739,  0.02704247, -0.19548259],
				  [	0.09049679  ,0.03337559 , 0.99533733, -0.60420346],
				  [ 0.  ,        0. ,         0.,          1.    ]])
	A = np.array([[ 2.66904865e-01,  9.62905162e-01,  3.96918440e-02,  2.02237555e-01],
	 			  [-9.63722849e-01,  2.66689146e-01,  1.07316972e-02,  5.50751884e-01],
	              [-2.51777394e-04, -4.11162791e-02,  9.99154337e-01, -5.34076005e-01],
	              [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
