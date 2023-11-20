#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import socket
import time
import signal
import sys

class LidarEmulatorNode(Node): # MODIFY NAME

    def __init__(self):
        super().__init__('lidar_emulator_node') # MODIFY NAME
        self.get_logger().info("Lidar Emulator Node has been started.")

    def send_file_contents_slow(self,client_socket, file_path, chunk_size=88, delay=0.0109):
        with open(file_path, 'rb') as file:
            while True:
                content = file.read(chunk_size)
                if not content:
                    break

                client_socket.sendall(content)
                time.sleep(delay)

    def start_server_slow(self,host, port, file_path):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
        server_socket.bind((host, port))
        server_socket.listen(1)

        print(f"Server listening on {host}:{port}")

        try:
            while True:
                print("Waiting for a connection...")
                client_socket, client_address = server_socket.accept()
                print(f"Accepted connection from {client_address}")

                try:
                    while True:
                        # Send the file contents to the client with a delay
                        self.send_file_contents_slow(client_socket, file_path)
                        print("File sent successfully.")
                except Exception as e:
                    print(f"Error sending file: {e}")
                finally:
                    # Close the connection
                    client_socket.close()
        except KeyboardInterrupt:
            print("Ctrl+C pressed. Exiting gracefully.")
        finally:
            # Close the server socket
            server_socket.close()


def main(args=None):
    rclpy.init(args=args)
    node = LidarEmulatorNode() # MODIFY NAME
   
    # Replace 'your_file_path' with the path to your binary file
    file_path = 'lidar_home.txt'
    host = '127.0.0.1'  # Use '0.0.0.0' to listen on all available interfaces
    port = 2327  # Choose a suitable port number

    node.start_server_slow(host, port, file_path)
    
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
