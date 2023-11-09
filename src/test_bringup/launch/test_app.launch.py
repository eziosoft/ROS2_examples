from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    test_publisher_node = Node(
        package='test_pkg',
        executable='test_publisher',
        name='test_publisher'
    )

    test_subscriber_node = Node(
        package='test_pkg',
        executable='test_subscriber',
        name='test_subscriber'
    )

    ld.add_action(test_publisher_node)
    ld.add_action(test_subscriber_node)

    return ld
