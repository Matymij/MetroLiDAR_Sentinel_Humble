from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bag = LaunchConfiguration('bag')
    use_bag = LaunchConfiguration('use_bag')
    use_player = LaunchConfiguration('use_player')
    data = LaunchConfiguration('data_dir')

    return LaunchDescription([
        DeclareLaunchArgument('bag', default_value=''),
        DeclareLaunchArgument('use_bag', default_value='false'),
        DeclareLaunchArgument('use_player', default_value='false'),
        DeclareLaunchArgument('data_dir', default_value='/data'),

        # Python preprocessor вместо C++ (большие сообщения проходят без потерь)
        ExecuteProcess(
            cmd=[
                'python3', '/ws/scripts/preprocessor_py.py',
                '--ros-args', '--params-file', '/ws/config/params.yaml',
            ],
            output='screen',
        ),

        Node(
            package='metro_obstacle_detector',
            executable='detector_node',
            name='obstacle_detector',
            output='screen',
            parameters=['/ws/config/params.yaml'],
        ),

        Node(
            package='lidar_dataset_player',
            executable='player',
            name='lidar_dataset_player',
            output='screen',
            parameters=[{'data_dir': data}],
            condition=IfCondition(use_player),
        ),

        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag, '--clock'],
            output='screen',
            condition=IfCondition(use_bag),
        ),
    ])