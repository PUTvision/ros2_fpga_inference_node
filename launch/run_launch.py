from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_fpga_inference_node',
            namespace='ros2_fpga_inference_node',
            executable='image_publisher',
            name='sim'
        ),
        Node(
            package='ros2_fpga_inference_node',
            namespace='ros2_fpga_inference_node',
            executable='inference_engine',
            name='sim'
        )
    ])