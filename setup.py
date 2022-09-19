from setuptools import setup

package_name = 'ros2_fpga_inference_node'
submodules = 'ros2_fpga_inference_node/inference_engine'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='petalinux',
    maintainer_email='petalinux@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher = ros2_fpga_inference_node.camera_image_publisher:main',
            'inference_engine = ros2_fpga_inference_node.inference_wrapper_node:main'
            ],
    },
)
