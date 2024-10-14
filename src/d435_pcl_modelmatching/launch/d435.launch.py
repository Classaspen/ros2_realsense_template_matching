from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
   return LaunchDescription([
      Node(
         package='d435_pcl_modelmatching',
         executable='d435_pcl_modelmatching_node',
         name='d435_pcl_modelmatching_node'
      ),
   ])