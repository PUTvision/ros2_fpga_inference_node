import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

from inference_node.inference_runner import InferenceRunner


class InferenceWrapperNode(Node):
    def __init__(self):
        super().__init__('inference_wrapper_node')

        self.model_path = get_package_share_directory('inference_node') + '/data/UNET_int_VCK190_v2.xmodel'
        self.inference_runner = InferenceRunner(self.model_path)

        self.segmentation_publisher = self.create_publisher(CompressedImage, '/inference/compressed', 1)

        self.image_listener = self.create_subscription(CompressedImage, '/color/compressed', self.processing_callback, 1)

        self.get_logger().info('Waiting for image!')

    @staticmethod
    def sigmoid(x):
        return 1 / (1 + np.exp(-x.astype(np.float32)))

    def processing_callback(self, msg: CompressedImage):
        self.get_logger().info('I got image!')
        img = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_UNCHANGED)

        img = cv2.resize(img, (640,360))

        # make padding to 32x
        inp = np.zeros((384,640,3), dtype=np.uint8)
        inp[:360,:640, :] = img

        inp = inp.astype(np.float32) * 1./255.

        out = self.inference_runner.predict(inp).squeeze()
        out = self.sigmoid(out)
        
        out = np.uint8(out*255)

        self.publish_segmentation(out[:360,:640], msg.header)

    def publish_segmentation(self, mask: np.ndarray, header):
        msg = CompressedImage()
        msg.header = header
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', mask)[1]).tostring()

        self.segmentation_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    inference = InferenceWrapperNode()
    rclpy.spin(inference)
    
    inference.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
