import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    params_path = os.path.join(
        get_package_share_directory('motion_planner'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='gui',
            executable='simulator',
            name='simulator',
            output='screen'
        ),
        Node(
            package='motion_planner',
            executable='motion_planner',
            name='motion_planner',
            output='screen',
            parameters=[params_path]
        ),
    ])
