import open3d as o3d

# 读取 PLY 文件
pcd = o3d.io.read_point_cloud("/home/classaspen/realsense_ros_4.0.4/src/model/source_model_ply/pointcloudcumb.ply")

# 将点云保存为 PCD 文件
o3d.io.write_point_cloud("/home/classaspen/realsense_ros_4.0.4/src/model/new_model_pcd/pointcloudcumb.pcd", pcd)
