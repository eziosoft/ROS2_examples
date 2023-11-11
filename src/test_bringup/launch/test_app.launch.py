from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # remap topic name
    remap_topic = ('/test_topic', '/test_topic_remap')

    test_publisher_node = Node(
        package='test_pkg',
        executable='test_publisher',
        name='test_publisher',
        remappings=[remap_topic]
    )

    test_subscriber_node = Node(
        package='test_pkg',
        executable='test_subscriber',
        name='test_subscriber',
        remappings=[remap_topic]
    )

    ld.add_action(test_publisher_node)
    ld.add_action(test_subscriber_node)

    return ld
