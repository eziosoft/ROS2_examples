import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    remap_topic = ('/scan', '/scan_l')

    config_dir = '/home/e/ros2_ws/src/test_bringup/config'

    config_basename = 'backpack_2d.lua'
    # config_basename = 'turtlebot.lua'

    return LaunchDescription([

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'laser', 'map','10'],
        ),

        #  Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher',
        #     output='screen',
        #     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'map','10'],
        # ),

        #  Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher',
        #     output='screen',
        #     arguments=['0', '0', '0', '0', '0', '0',  'base_link' 'odom','100'],
        # ),


        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='mapping_node',
            output='screen',
            arguments=['-configuration_directory', config_dir,
                       '-configuration_basename', config_basename],
            # remappings=[remap_topic]
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_node',

        ),
    ])
