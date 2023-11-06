#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class TestNode(Node): # MODIFY NAME

    def __init__(self):
        super().__init__('test_node') # MODIFY NAME
        self.get_logger().info("Hello World!")
        self.create_timer(1.0, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        self.get_logger().info('Hello World! %d' % self.i)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = TestNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
