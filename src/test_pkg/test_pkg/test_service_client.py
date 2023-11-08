#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts


class AddTwoIntsClient(Node):  # MODIFY NAME

    def __init__(self):
        super().__init__('add_two_ints_client')  # MODIFY NAME
        self.call_add_two_ints_server(3, 4)

    def call_add_two_ints_server(self, a, b):
        self.get_logger().info('Add Two Ints Client has been started')
        client = self.create_client(AddTwoInts, 'add_two_ints')
        while not client.wait_for_service(1.0):
            self.get_logger().warn('Waiting for Server')

        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = client.call_async(request)
        future.add_done_callback(self.future_callback)

    def future_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Result of add two ints: %d' % response.sum)
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClient()  # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
