#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# Remember to add new dependencies to package.xml
from example_interfaces.msg import String


class TestPublisher(Node):  # MODIFY NAME

    def __init__(self):
        super().__init__('test_publisher')  # MODIFY NAME
        self.declare_parameter(name='start_number', value=0)


        self.create_timer(1.0, self.publish_test)
        self.i = self.get_parameter('start_number').value

        self.publisher = self.create_publisher(String, 'test_topic', 10) # (msg type, topic name, queue size)

        self.get_logger().info("Test Publisher Node has been started")

    def publish_test(self):
        self.i += 1

        msg = String()
        msg.data = "Hello wold message %d" % self.i

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()  # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
