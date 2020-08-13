import open3d as o3d
import numpy as np

def get_point_cloud(file):
	return o3d.io.read_point_cloud(file)
file = "camera_1.ply"
pc = get_point_cloud(file)	
axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size = 1, origin = np.array([0,0,0]))
o3d.visualization.draw_geometries([pc,axes])