#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from example_interfaces.msg import String


class TestSubscriber(Node):  # MODIFY NAME

    def __init__(self):
        super().__init__('test_subscriber')  # MODIFY NAME

        self.subscription = self.create_subscription(
            String,
            'test_topic',
            self.listener_callback,
            10)

        self.get_logger().info("Test Subscriber Node has been started")

    def listener_callback(self, msg):
        self.get_logger().info("I heard: [%s]" % msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = TestSubscriber()  # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
