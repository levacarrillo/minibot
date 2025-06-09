from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument 
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition #, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Launch in simulation or real mode'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('hardware'),
                    'launch',
                    'hardware.launch.py'
                ])
            ]),
            launch_arguments={'use_sim': use_sim}.items()
        ),
        Node(
            package='mobile_base',
            executable='mobile_base_simul',
            name='mobile_base_simul',
            output='screen',
            condition=IfCondition(use_sim)
        ),
    ])