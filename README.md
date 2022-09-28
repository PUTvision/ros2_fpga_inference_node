* Append to `.bashrc`:
  ```
  export ROS_OS_OVERRIDE=openembedded
  export ROS_DISTRO=foxy
  export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/usr/lib/"
  export PKG_CONFIG_PATH="/usr/lib/pkgconfig:/usr/share/pkgconfig"
  
  source /usr/bin/ros_setup.bash
  ```

* Prepare
  ```
  sudo rosdep init
  rosdep update
  ```
  
  
* Create package
  ```
  mkdir -p ~/ros2_ws/src
  cd ~/ros2_ws/src
  git clone https://github.com/PUTvision/ros2_fpga_inference_node.git
  cd ~/ros2_ws
  rosdep install -i --from-path src --rosdistro foxy -y
  colcon build
  source install/local_setup.bash
  ```
