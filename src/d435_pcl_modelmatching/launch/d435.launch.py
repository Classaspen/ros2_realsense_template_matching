from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
    ])