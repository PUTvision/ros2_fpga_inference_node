import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

from xilinx_runner.inference_runner import InferenceRunner


class InferenceWrapperNode(Node):
    def __init__(self):
        super().__init__('inference_wrapper_node')

        self.model_path = get_package_share_directory('inference_node') + '/data/unet_1xC32B1L2S2_640x360.xmodel'
        self.inference_runner = InferenceRunner(self.model_path)

        self.segmentation_publisher = self.create_publisher(CompressedImage, '/inference', 1)

        self.image_listener = self.create_subscription(CompressedImage, '/color/compressed', self.processing_callback, 1)

        self.get_logger().info('Waiting for image!')

    def processing_callback(self, msg: CompressedImage):
        self.get_logger().info('I got image!')
        img = cv2.imdecode(np.fromstring(ros_data.data, np.uint8), cv2.IMREAD_COLOR)

        out = self.inference_runner.run_network(img).squeeze()

        self.publish_segmentation(out)

    def publish_segmentation(mask: np.ndarray):
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()

        self.segmentation_publisher.pub(msg)

def main(args=None):
    rclpy.init(args=args)

    inference = InferenceWrapperNode()
    rclpy.spin(inference)
    
    inference.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
