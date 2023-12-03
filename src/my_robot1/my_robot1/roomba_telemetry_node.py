#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from math import sin, cos, pi

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Int16

from my_robot1.roomba_telemetry import OdonometryDriver
from my_robot1.create2_enums import Specs

NS_TO_SEC = 1000000000


class RoombaTelemetryNode(Node):  # MODIFY NAME

    def __init__(self):
        super().__init__('roomba_telemetry_node')  # MODIFY NAME
        self.odom_driver = OdonometryDriver("192.168.8.103", 1883)

        self.roomba_status_check_hz = 10.0
        self.timer_update_callback = self.create_timer(
            1.0/self.roomba_status_check_hz, self.timer_update_callback)



        ### parameters #######
        self.rate_hz = self.declare_parameter("rate_hz", 5.0).value # the rate at which to publish the transform
        self.create_timer(1.0/self.rate_hz, self.update)

        self.ticks_meter = float(
            self.declare_parameter('ticks_meter', Specs.TicksPerMeter).value)  # The number of wheel encoder ticks per meter of travel
        self.base_width = float(self.declare_parameter('base_width', Specs.WheelDistanceInM).value)  # The wheel base width in meters

        self.base_frame_id = self.declare_parameter('base_frame_id',
                                                    'base_footprint').value  # the name of the base frame of the robot
        self.odom_frame_id = self.declare_parameter('odom_frame_id',
                                                    'odom').value  # the name of the odometry reference frame

        self.encoder_min = self.declare_parameter('encoder_min', -32768).value
        self.encoder_max = self.declare_parameter('encoder_max', 32768).value
        self.encoder_low_wrap = self.declare_parameter('wheel_low_wrap', (
                self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min).value
        self.encoder_high_wrap = self.declare_parameter('wheel_high_wrap', (
                self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min).value

        # internal data
        self.enc_left = None  # wheel encoder readings
        self.enc_right = None
        self.left = 0.0  # actual values coming back from robot
        self.right = 0.0
        self.lmult = 0.0
        self.rmult = 0.0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.x = 0.0  # position in xy plane
        self.y = 0.0
        self.th = 0.0
        self.dx = 0.0  # speeds in x/rotation
        self.dr = 0.0
        self.then = self.get_clock().now()

        # subscriptions
        # self.create_subscription(Int16, "lwheel", self.lwheel_callback, 10)
        # self.create_subscription(Int16, "rwheel", self.rwheel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.odom_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(f"ticks_meter = {self.ticks_meter}")
        self.get_logger().info(f"base_width = {self.base_width}")
        self.get_logger().info(f"base_frame_id = {self.base_frame_id}")
        self.get_logger().info(f"odom_frame_id = {self.odom_frame_id}")
        self.get_logger().info(f"encoder_min = {self.encoder_min}")
        self.get_logger().info(f"encoder_max = {self.encoder_max}")
        self.get_logger().info(f"encoder_low_wrap = {self.encoder_low_wrap}")
        self.get_logger().info(f"encoder_high_wrap = {self.encoder_high_wrap}")

        self.get_logger().info("Roomba Telemetry Node has been started")
        self.left = 0
        self.right = 0


    def update(self):
        now = self.get_clock().now()
        elapsed = now - self.then
        self.then = now
        elapsed = elapsed.nanoseconds / NS_TO_SEC

        # calculate odometry
        if self.enc_left == None:
            d_left = 0
            d_right = 0
        else:
            d_left = (self.left - self.enc_left) / self.ticks_meter
            d_right = (self.right - self.enc_right) / self.ticks_meter
        self.enc_left = self.left
        self.enc_right = self.right

        # distance traveled is the average of the two wheels 
        d = (d_left + d_right) / 2
        # this approximation works (in radians) for small angles
        th = (d_right - d_left) / self.base_width
        # calculate velocities
        self.dx = d / elapsed
        self.dr = th / elapsed

        if d != 0:
            # calculate distance traveled in x and y
            x = cos(th) * d
            y = -sin(th) * d
            # calculate the final position of the robot
            self.x = self.x + (cos(self.th) * x - sin(self.th) * y)
            self.y = self.y + (sin(self.th) * x + cos(self.th) * y)
        if th != 0:
            self.th = self.th + th

        # publish the odom information
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin(self.th / 2)
        quaternion.w = cos(self.th / 2)

        transform_stamped_msg = TransformStamped()
        transform_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        transform_stamped_msg.header.frame_id = self.base_frame_id
        transform_stamped_msg.child_frame_id = self.odom_frame_id
        transform_stamped_msg.transform.translation.x = self.x
        transform_stamped_msg.transform.translation.y = self.y
        transform_stamped_msg.transform.translation.z = 0.0
        transform_stamped_msg.transform.rotation.x = quaternion.x
        transform_stamped_msg.transform.rotation.y = quaternion.y
        transform_stamped_msg.transform.rotation.z = quaternion.z
        transform_stamped_msg.transform.rotation.w = quaternion.w

        self.odom_broadcaster.sendTransform(transform_stamped_msg)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion
        odom.child_frame_id = self.base_frame_id
        odom.twist.twist.linear.x = self.dx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.dr
        self.odom_pub.publish(odom)

    def lwheel_callback(self, enc):
        if enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap:
            self.lmult = self.lmult + 1

        if enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap:
            self.lmult = self.lmult - 1

        self.left = 1.0 * (enc + self.lmult * (self.encoder_max - self.encoder_min))
        self.prev_lencoder = enc

    def rwheel_callback(self, enc):
        if enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap:
            self.rmult = self.rmult + 1

        if enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap:
            self.rmult = self.rmult - 1

        self.right = 1.0 * (enc + self.rmult * (self.encoder_max - self.encoder_min))
        self.prev_rencoder = enc
























    def timer_update_callback(self):
        # self.get_logger().info("Timer update callback")
        state = self.odom_driver.get_state()
        if (
            state is not None
            and hasattr(state, "leftEncoderCounts")
            and hasattr(state, "rightEncoderCounts")
        ):
            left = state.leftEncoderCounts
            right = state.rightEncoderCounts
            if left != self.left or right != self.right:
                self.left = left
                self.right = right
                self.calculate_odonometry()

        # if self.left > 32768:
        #     self.left = -32768

        # if self.right > 32768:
        #     self.right = -32768
            
        # self.left+=10
        # self.right-=10
        # self.calculate_odonometry()

    def calculate_odonometry(self):
        self.get_logger().info("Left: %d, Right: %d" % (self.left, self.right))
        self.lwheel_callback(self.left)
        self.rwheel_callback(self.right)


def main(args=None):
    rclpy.init(args=args)
    node = RoombaTelemetryNode()  # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
