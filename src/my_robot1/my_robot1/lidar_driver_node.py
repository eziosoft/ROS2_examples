#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from my_robot1.lidar_client import LidarClient, LidarOutput

from sensor_msgs.msg import LaserScan


class LidarNode(Node):  # MODIFY NAME
    TOPIC_NAME = 'scan'

    def __init__(self):
        super().__init__('lidar_node1')  # MODIFY NAME

        IP = self.declare_parameter("ip", "Lidar").value
        PORT = self.declare_parameter("port", 23).value
        self.get_logger().info(f"Connecting to {IP}:{PORT}")

        self.mesurements = []
        self.angles = []
        self.last_angle = 0
        self.last_time = self.get_clock().now()

        self.lidar = LidarClient(IP, PORT, callback=self.on_data)
        self.lidar.connect_to_server()

        self.scan_publisher = self.create_publisher(
            LaserScan, self.TOPIC_NAME, 10)
        self.get_logger().info("Lidar Node has been started.")
        self.get_logger().info(f"Publishing to {self.TOPIC_NAME} topic.")

        self.lidar.start_listening()

    def on_data(self, lidarOutput: LidarOutput):

        if lidarOutput.error:
            self.get_logger().error(lidarOutput.errorMsg)
            return

        for i in range(len(lidarOutput.distances)):
            if lidarOutput.angles[i] < self.last_angle and lidarOutput.angles[i] in range(0, 22):
                scan_time = 1/lidarOutput.speed_rev_s
                time_increment = scan_time/len(self.mesurements)

                self.publish(angle_min=0,#self.angles[0],
                             angle_max=360,#self.angles[-1],
                             angle_increment=self.angles[1]-self.angles[0],
                             ranges=self.mesurements,
                             scan_time=scan_time,
                             time_increment=time_increment)
                self.mesurements = []
                self.angles = []
                self.last_angle = lidarOutput.angles[i]
            else:
                self.mesurements.append(lidarOutput.distances[i])
                self.angles.append(lidarOutput.angles[i])
                self.last_angle = lidarOutput.angles[i]

    def publish(self, angle_min, angle_max, angle_increment, ranges, scan_time, time_increment):
        msg = LaserScan()
        msg.header.frame_id = "base_scan"
        time = self.get_clock().now()
        msg.header.stamp = time.to_msg()
        msg.angle_min = math.radians(angle_min)
        msg.angle_max = math.radians(angle_max)
        msg.angle_increment = math.radians(angle_increment)
        msg.time_increment = time_increment
        msg.scan_time = scan_time
        msg.range_min = 0.1
        msg.range_max = 8.0
        msg.ranges = ranges
        msg.intensities = [0.0] * len(ranges)

        self.scan_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()  # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
