from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
import os
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('realsense2_camera'), 'launch'))
import rs_launch

local_parameters = [{'name': 'camera_name',                  'default': 'camera', 'description': 'camera unique name'},
                    {'name': 'camera_namespace',             'default': 'camera', 'description': 'camera namespace'},
                    {'name': 'enable_color',                 'default': 'true', 'description': 'enable color stream'},
                    {'name': 'enable_depth',                 'default': 'true', 'description': 'enable depth stream'},
                    {'name': 'pointcloud.enable',            'default': 'true', 'description': 'enable pointcloud'},
                   ]

def set_configurable_parameters(local_params):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in local_params])

def generate_launch_description():
    params = rs_launch.configurable_parameters
    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) +
        rs_launch.declare_configurable_parameters(params) + 
        [
        OpaqueFunction(function=rs_launch.launch_setup,
                kwargs = {'params' : set_configurable_parameters(params)}
        ),
        Node(
            package='d435_pcl_modelmatching',  # 包名
            executable='d435_pcl_modelmatching_node',  # 可执行节点名
            name='d435_pcl_modelmatching_node',  # 节点名称
            output='screen',  # 输出到终端
            parameters=[  
               #  {'param_name1': param_value1},  # 示例参数
               #  {'param_name2': param_value2},
            ],
            remappings=[  # 主题重映射
                ('/original_topic', '/remapped_topic'),
            ],
        ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [ThisLaunchFileDir(), '/rviz/pointcloud.rviz']]
    )
    ])