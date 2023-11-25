import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = '/home/e/ros2_ws/src/my_robot1_bringup/config'

    config_basename = 'slam_2d.lua'

    return LaunchDescription([

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='robot_transform_publisher',
            output='screen',
            arguments=['0.0', '0.0', '0.2', '0.0',
                       '0.0', '0.0', 'base_link', 'base_scan'],
        ),


        Node(
            package='my_robot1',
            executable='lidar_emulator',
            name='lidar_emulator_node',
            output='screen',
        ),

        Node(
            package='my_robot1',
            executable='lidar_driver',
            name='lidar_driver_node',
            output='screen',
            parameters=[{'ip': '127.0.0.1'},
                        {'port': 2323}]
        ),


        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='mapping_node',
            output='screen',
            arguments=['-configuration_directory', config_dir,
                       '-configuration_basename', config_basename],
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_node',

        ),

        Node(
            package='my_robot1',
            executable='drive_node',
            name='drive_node',
            output='screen',
        )
    ])
