#!/usr/bin/env python3

import math
import random
import time
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from test_pkg.MyLidar import LidarClient, LidarOutput

from sensor_msgs.msg import LaserScan



class LidarNode(Node):  # MODIFY NAME
    TOPIC_NAME = 'scan'

    def __init__(self):
        super().__init__('lidar_node')  # MODIFY NAME

        IP = '127.0.0.1'
        PORT = 2325
        self.lidar = LidarClient(IP, PORT, callback=self.on_data)
        self.lidar.connect_to_server()

        self.scan_publisher = self.create_publisher(LaserScan, self.TOPIC_NAME, 10)
        self.get_logger().info("Lidar Node has been started.")
        self.get_logger().info(f"Publishing to {self.TOPIC_NAME} topic.")

        self.lidar.start_listening()

    def on_data(self, lidarOutput: LidarOutput):

        if lidarOutput.error:
            self.get_logger().error(lidarOutput.errorMsg)
            return
        
        msg = LaserScan()
        msg.header.frame_id = "laser"
        time = self.get_clock().now()
        msg.header.stamp = time.to_msg() 
        msg.angle_min = math.radians(lidarOutput.angStart)
        msg.angle_max = math.radians(lidarOutput.angStart +
                                     lidarOutput.nSamples * lidarOutput.angOffset)
        msg.angle_increment = math.radians(lidarOutput.angOffset)
        
        number_of_frames_per_revolution = 360 / 22.5
        msg.time_increment = 1.0 / lidarOutput.speed_rev_s / (lidarOutput.nSamples * number_of_frames_per_revolution)
        
        msg.scan_time = 1.0 / (lidarOutput.speed_rev_s * number_of_frames_per_revolution)
        msg.range_min = 0.1
        msg.range_max = 8.0
        msg.ranges = lidarOutput.distances

        self.scan_publisher.publish(msg)
        


def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()  # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
