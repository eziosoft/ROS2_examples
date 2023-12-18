import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    map_file = '/home/e/ros2_ws/map/my_map.yaml'
    param_file = '/home/e/ros2_ws/src/my_robot1_bringup/config/nav_params1.yaml'

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory(
                    'nav2_bringup'), '/launch', '/bringup_launch.py']
            ),
            launch_arguments={
                'map':map_file,
                'param_file':param_file
            }.items()

        ),

       
    ])
