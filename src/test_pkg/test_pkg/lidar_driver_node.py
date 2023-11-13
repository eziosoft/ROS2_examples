#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from test_pkg.MyLidar import MyLidar, LidarOutput


class LidarNode(Node):  # MODIFY NAME

    def __init__(self):
        super().__init__('lidar_node')  # MODIFY NAME
        self.lidar = MyLidar()
        connected = self.lidar.connect()
        self.get_logger().info("Lidar Node has been started.")

        if connected:
            self.get_logger().info("Lidar connected.")
        else:
            self.get_logger().error("Lidar not connected.")

        


def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()  # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
