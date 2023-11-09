#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from test_interfaces.msg import TestStatus

class TestStatusPublisher(Node): # MODIFY NAME

    def __init__(self):
        super().__init__('test_status_publisher') # MODIFY NAME
        self.publisher = self.create_publisher(TestStatus, 'test_status', 10)
        self.create_timer(1.0, self.publishTestStatus)
        self.get_logger().info("Test Status Publisher Node has been started")

    def publishTestStatus(self):
        msg = TestStatus()
        msg.test_value = 1
        msg.debug_message = "Hello wold message"
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TestStatusPublisher() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from test_interfaces.msg import TestStatus

class TestStatusPublisher(Node): # MODIFY NAME

    def __init__(self):
        super().__init__('test_status_publisher') # MODIFY NAME
        self.publisher = self.create_publisher(TestStatus, 'test_status', 10)
        self.create_timer(1.0, self.publishTestStatus)
        self.get_logger().info("Test Status Publisher Node has been started")

    def publishTestStatus(self):
        msg = TestStatus()
        msg.value1 = 1.1
        msg.value2 = "Hello wold message"
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TestStatusPublisher() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
