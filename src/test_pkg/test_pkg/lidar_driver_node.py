#!/usr/bin/env python3

import math
import random
import rclpy
from rclpy.node import Node

from test_pkg.MyLidar import LidarClient, LidarOutput

from sensor_msgs.msg import LaserScan


class LidarNode(Node):  # MODIFY NAME

    def __init__(self):
        super().__init__('lidar_node')  # MODIFY NAME

        IP = '192.168.8.200'
        PORT = 23
        self.lidar = LidarClient(IP, PORT, callback=self.on_data)
        self.lidar.connect_to_server()

        self.scan_publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.get_logger().info("Lidar Node has been started.")

        self.lidar.start_listening()

        # self.create_timer(0.2, self.publish_emulated_scan)

    def on_data(self, lidarOutput: LidarOutput):
        msg = LaserScan()
        msg.header.frame_id = "laser"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.angle_min = math.radians(lidarOutput.angStart)
        msg.angle_max = math.radians(lidarOutput.angStart +
                                     lidarOutput.nSamples * lidarOutput.angOffset)
        msg.angle_increment = math.radians(lidarOutput.angOffset)
        msg.time_increment = 1000.0 / lidarOutput.speed / lidarOutput.nSamples
        msg.scan_time = 1000.0 / lidarOutput.speed
        msg.range_min = 0.0
        msg.range_max = 10.0
        msg.ranges = lidarOutput.distances

        self.scan_publisher.publish(msg)
        self.get_logger().info("Scan published.")

    # def publish_emulated_scan(self):
    #     circle_ranges = []
    #     for i in range(0, 63):
    #         circle_ranges.append(2.0 + random.random() / 10.0)

    #     msg = LaserScan()
    #     msg.header.frame_id = "laser"
    #     msg.header.stamp = self.get_clock().now().to_msg()
    #     msg.angle_min = 0.0
    #     msg.angle_max = 6.28
    #     msg.angle_increment = 0.1
    #     msg.time_increment = 0.001
    #     msg.scan_time = 0.05
    #     msg.range_min = 0.0
    #     msg.range_max = 10.0
    #     msg.ranges = circle_ranges

    #     self.scan_publisher.publish(msg)
    #     self.get_logger().info("Scan published.")


def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()  # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
