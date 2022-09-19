import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image

from .inference_engine.resnet50_engine import Resnet50Engine


class InferenceWrapperNode(Node):

    def __init__(self):
        super().__init__('inference_wrapper_node')
        self.engine = Resnet50Engine(
            '/home/petalinux/TEST/resnet_vck190/tf2_resnet50_imagenet_224_224_7.76G_2.5.xmodel',
            '/home/petalinux/TEST/imagenet1000_clsidx_to_labels.txt'
        )

        self.publisher_ = self.create_publisher(String, '/inference', 1)

        self.image_subscriptor_ = self.create_subscription(Image, '/camera/raw', self.processing_callback, 1)

        self.get_logger().info('Waiting for image!')

    def processing_callback(self, msg: Image):
        self.get_logger().info('I got image!')
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)

        out = self.engine.infer(img)

        msg = String()
        msg.data = self.engine.cls_to_name(out)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    inference = InferenceWrapperNode()
    rclpy.spin(inference)
    
    inference.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
