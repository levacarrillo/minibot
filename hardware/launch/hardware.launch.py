import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument 
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition #, UnlessCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    params_path = os.path.join(
        get_package_share_directory('hardware'),
        'config',
        'params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='hardware',
            executable='lidar_sensor',
            name='lidar_sensor',
            output='screen',
        ),
        Node(
            package='hardware',
            executable='light_sensors',
            name='light_sensors',
            output='screen',
        ),
        Node(
            package='hardware',
            executable='robot_introspection',
            name='robot_introspection',
            output='screen',
            parameters=[params_path],
        ),
    ])
