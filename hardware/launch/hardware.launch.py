import os
import xacro
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument 
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition #, UnlessCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim')

    xacro_file = os.path.join(
        get_package_share_directory('hardware'),
        'urdf',
        'minibot.urdf.xacro'
    )

    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    rviz_config_file = os.path.join(
        get_package_share_directory('hardware'),
        'rviz',
        'display.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Launch nodes for simulation if true is setted'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[robot_description, {'use_sim_time': use_sim}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            condition=IfCondition(use_sim)
        ),
        Node(
            package='hardware',
            executable='lidar_simulator',
            name='lidar_simulator',
            output='screen',
            condition=IfCondition(use_sim)
        ),
        Node(
            package='hardware',
            executable='light_sensors_simulator',
            name='light_sensors_simulator',
            output='screen',
            condition=IfCondition(use_sim)
        ),
        # Node(
        #     package='rqt_graph',
        #     executable='rqt_graph',
        #     name='rqt_graph',
        #     output='screen',
        #     condition=IfCondition(use_sim)
        # ),
    ])
