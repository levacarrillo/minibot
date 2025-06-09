from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument 
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition #, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim')

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
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            prefix = 'xterm -e',
            output='screen',
        ),
    ])