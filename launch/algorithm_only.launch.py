from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
 return LaunchDescription([Node(package='lidar_preprocessor',executable='pointcloud_preprocessor',output='screen',parameters=['/ws/config/params.yaml']),Node(package='metro_obstacle_detector',executable='detector_node',output='screen',parameters=['/ws/config/params.yaml'])])
