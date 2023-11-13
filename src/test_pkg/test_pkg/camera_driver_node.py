import time
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraDriverNode(Node):
    def __init__(self):
        super().__init__('camera_driver_node')
        self.declare_parameter('camera_url', 'http://192.168.8.229:5000')
        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()
        self.cap = None
        self.stream_connected = False  # Flag to indicate stream connection status
        self.get_logger().info('Camera driver node has been started.')
        self.start_publishing()

    def start_publishing(self):
        while rclpy.ok():
            try:
                self.cap = cv2.VideoCapture(self.get_parameter('camera_url').value)
                while self.cap.isOpened():
                    ret, frame = self.cap.read()
                    if ret:
                        if not self.stream_connected:
                            self.get_logger().info('Stream connection restored.')
                            self.stream_connected = True  # Update the flag
                        # Convert OpenCV image to ROS Image message
                        ros_image = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                        self.publisher.publish(ros_image)
                    else:
                        self.get_logger().error('Error reading frame from camera.')
                        time.sleep(5)
                        self.stream_connected = False  # Update the flag
                        break  # Break out of the inner loop to retry connection
            except Exception as e:
                self.get_logger().error(f'Error: {str(e)}')
                time.sleep(5)
                self.stream_connected = False  # Update the flag
            finally:
                if self.cap is not None:
                    self.cap.release()

    def stop_publishing(self):
        if self.cap is not None:
            self.cap.release()

    def __del__(self):
        self.stop_publishing()

def main(args=None):
    rclpy.init(args=args)
    camera_driver_node = CameraDriverNode()
    rclpy.spin(camera_driver_node)
    camera_driver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
