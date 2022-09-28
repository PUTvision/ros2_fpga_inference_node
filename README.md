* Append to `.bashrc`:
  ```
  export ROS_OS_OVERRIDE=openembedded
  export ROS_DISTRO=foxy
  export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/usr/lib/"
  export PKG_CONFIG_PATH="/usr/lib/pkgconfig:/usr/share/pkgconfig"
  
  source /usr/bin/ros_setup.bash
  ```
