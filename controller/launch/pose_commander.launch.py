import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument 
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim')

    params_path = os.path.join(
        get_package_share_directory('controller'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='True to launch for simulation or false for real mode'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('mobile_base'),
                    'launch',
                    'mobile_base.launch.py'
                ])
            ]),
            launch_arguments={'use_sim': use_sim}.items()
        ),
        Node(
            package='controller',
            executable='pose_commander',
            name='pose_commander',
            output='screen',
            parameters=[params_path]
        ),
    ])