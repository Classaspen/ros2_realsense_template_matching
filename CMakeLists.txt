cmake_minimum_required(VERSION 3.5)

# 设置项目名称
project(RealSense)

# 添加子目录
add_subdirectory(src/d435_pcl_modelmatching)
add_subdirectory(src/realsense-ros/realsense2_camera)
add_subdirectory(src/realsense-ros/realsense2_description)
add_subdirectory(src/realsense-ros/realsense2_camera_msgs)
