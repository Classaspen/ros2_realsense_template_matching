cmake_minimum_required(VERSION 3.8)
project(d435_pcl_modelmatching)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "-fms-extensions")
MESSAGE([SEND_ERROR | STATUS | FATAL_ERROR] "message to display"
...)

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
# find_package(PCL COMPONENT REQUIRED)
# find_package(realsense2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PythonLibs REQUIRED)  

set(PCL_INCLUDE_DIRS /home/classaspen/pcl)

set(CMAKE_PREFIX_PATH "/opt/ros/humble")
set(DEPENDENCIES rclcpp std_msgs sensor_msgs)

# 
include_directories(/opt/ros/humble/include/pcl_conversions/)  
include_directories(/opt/ros/humble/include/message_filters/)  
include_directories(/opt/ros/humble/include/pcl_msgs/)  



include_directories(
    include
    SYSTEM
    rclcpp
    ${PYTHON_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    "/usr/include/eigen3" 
    "d435.h" 
    )

add_executable(d435_pcl_modelmatching_node 
  src/d435_pcl_modelmatching_node.cpp
  src/d435_pcl_processing.cpp
  )

  
# 添加库搜索路径
link_directories(/opt/ros/humble/lib/x86_64-linux-gnu/librealsense2.so)

# 链接 librealsense2 库
target_link_libraries(d435_pcl_modelmatching_node /opt/ros/humble/lib/x86_64-linux-gnu/librealsense2.so)



# 设置链接库路径
set(REALSENSE2_LIB_PATH /opt/ros/humble/lib/x86_64-linux-gnu)

target_link_libraries(d435_pcl_modelmatching_node 
  ${PYTHON_LIBRARIES}
  ${REALSENSE2_LIB_PATH}/librealsense2.so
  )

target_include_directories(d435_pcl_modelmatching_node PUBLIC
  src/include
  )

ament_target_dependencies(d435_pcl_modelmatching_node ${DEPENDENCIES} sensor_msgs)
target_compile_features(d435_pcl_modelmatching_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
INCLUDE_DIRECTORIES(d435_pcl_modelmatching_node${PCL_INCLUDE_DIRS})
# TARGET_LINK_LIBRARIES(d435_pcl_modelmatching_node /usr/local/include/pcl-1.12)
target_link_libraries(d435_pcl_modelmatching_node realsense2;${PCL_LIBRARIES})
# target_link_libraries(d435_pcl_modelmatching_node ${PCL_LIBRARIES})


install(TARGETS 
    d435_pcl_modelmatching_node
    DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()

# # ----------------------
# # 设置依赖
# set(DEPENDENCIES
#     rclcpp
#     std_msgs
#     PCL
# )

# # 添加依赖 
# ament_target_dependencies(d435_pcl_modelmatching_node ${DEPENDENCIES})

