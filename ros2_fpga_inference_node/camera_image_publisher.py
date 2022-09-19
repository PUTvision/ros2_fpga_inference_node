import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image


class CameraImagePublisher(Node):

    def __init__(self):
        super().__init__('camera_image_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/raw', 1)        
        self.cap = cv2.VideoCapture(5)

    def loop(self):
        while rclpy.ok():
            ret, frame = self.cap.read()

            if not ret:
                self.get_logger().info("Camera not returned frames")
                continue

            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.height = frame.shape[0]
            msg.width = frame.shape[1]
            msg.encoding = "rgb8"
            msg.is_bigendian = False
            msg.step = 3 * frame.shape[1]
            msg.data = frame.tobytes()

            self.publisher_.publish(msg)
            self.get_logger().info("Send image")


def main(args=None):
    rclpy.init(args=args)

    pub = CameraImagePublisher()
    pub.loop()
    
    pub.cap.release()
    pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
