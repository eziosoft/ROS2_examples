#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node

from test_pkg.mqtt_control import MqttControl
from geometry_msgs.msg import Twist


class DriveNode(Node):  # MODIFY NAME

    def __init__(self):
        super().__init__('drive_node')  # MODIFY NAME
        self.MQTT_BROKER_HOST = '192.168.0.19'
        self.MQTT_BROKER_PORT = 1883

        self.mqtt_control = MqttControl(
            self.MQTT_BROKER_HOST, self.MQTT_BROKER_PORT)
        self.mqtt_control.run()

        self.subscribtion = self.create_subscription(
            Twist,
            'cmd_vel',
            self.drive_callback,
            10)

        time.sleep(1)
        self.mqtt_control.enable_roomba()
        self.get_logger().info("Drive Node has been started.")

    def stop_roomba(self):
        self.mqtt_control.sendControl(0, 0, self.mqtt_control.COMMAND_STOP)
        self.get_logger().info("Roomba has been stopped.")
        time.sleep(1)

    def drive_callback(self, msg):
        self.get_logger().info("I heard: [%s]" % msg.linear.x)
        self.get_logger().info("I heard: [%s]" % msg.angular.z)

        x = int(msg.linear.x + 100)
        z = int(msg.angular.z + 100)

        if x > 200:
            x = 200
        if x < 0:
            x = 0
        if z > 200:
            z = 200
        if z < 0:
            z = 0

        self.mqtt_control.sendControl(z, x, 100)


def main(args=None):
    node = None
    try:
        rclpy.init(args=args)
        node = DriveNode()  # MODIFY NAME
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_roomba()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
