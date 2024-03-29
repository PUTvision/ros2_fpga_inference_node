# ROS2 Foxy node for computer vision tasks inference on Versal VCK190 together with Husky Rover and OAK-D Lite depth camera

<div align="center">
  <img src="./README_files/husky_on_the_moon.jpg" height="350px" />
 </div>

## Assumptions

The goal of Work Package 300 is to develop a framework that enables performing **deep learning inference** together with Robot Operating System (**ROS2**) directly on the Xilinx Versal **VCK190** developer board. Additionally, the framework can grab **color and depth** images from a commonly available sensor (in this case it is **Luxonis OAK-D Lite**) and publish them to ROS2 topics, and also can control the **Husky robot** using ROS2 messages.

## Hardware and software requirements

Below you can find the list of hardware and software that we use for this project.

### Software

* Vitis 2022.1
* Vitis HLS 2022.1
* Vivado 2022.1
* Petalinux 2022.1

### Hardware

* Host PC with Ubuntu 20.04.4 LTS.
* [Xilinx Versal VCK190](https://www.xilinx.com/products/boards-and-kits/vck190.html) developer board.
* [Luxonis OAK-D Lite](https://docs.luxonis.com/en/latest/pages/products/bw1098obc/) depth camera.
* [Husky robot](https://www.clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/) with [Ouster OS1-64](https://www.ouster.io/os1-64) lidar sensor.
* [IMU](https://www.xsens.com/products/mti-30-series) sensor.

### Development tools

* VS Code - good IDE for remote development.
* Tmux - terminal multiplexer, allows to detach and reattach to a running session.
* ROS2 Foxy - host installation enables the generation of ROS2 messages and services.
* Picocom - serial communication terminal.

## Workflow

### Create project and developer board device configuration

The main part of this step is based on the [VCK190 Base TRD](https://xilinx.github.io/vck190-base-trd/2022.1/html/index.html) document. It contains all the necessary information about the VCK190 board and how to create a project for it with working DPU inference. Follow 1 to 4.4 steps from the document and then continue with the steps below to generate the Petalinux image and copy it to an SD card.

> Note, that `$WORKSPACE` variable is the path to the petalinux project directory generated inside `vck190-base-trd`. In our case it is `/home/put/Projects/vck190-base-trd/petalinux/xilinx-vck190-base-trd/`.

```bash
# COPY DEDICATED VCK190 FILES TO THE PROJECT
cp $WORKSPACE/../../overlays/xvdpu/kernels/binary_container_1.xsa $WORKSPACE/project-spec/hw-description/system.xsa
cp $WORKSPACE/../../overlays/xvdpu/kernels/binary_container_1.xclbin $WORKSPACE/images/linux/
cp -r $WORKSPACE/../../overlays/xvdpu/apps/*  $WORKSPACE/project-spec/meta-base-trd/recipes-apps

# BUILD PROJECT
petalinux-build

# GENERATE BOOT FILES
petalinux-package --boot --u-boot --dtb $WORKSPACE/images/linux/system.dtb --qemu-rootfs no --force

# GENERATE BOOT IMAGE
petalinux-package --wic --bootfiles "binary_container_1.xclbin ramdisk.cpio.gz.u-boot boot.scr Image BOOT.BIN"

# COPY BOOT IMAGE TO SD CARD
sudo dd if=$WORKSPACE/images/linux/petalinux-sdimage.wic of=<DEVICE> status=progress conv=sync
```

### Petalinux configuration

Changes in the Petalinux configuration are required to enable some features. You can edit it using `petalinux-config` tool.

* Set MAC address for the ethernet interface.
    ```
    Subsystem AUTO Hardware Settings  --->
        Ethernet Settings  --->
            Disable: Randomise MAC address
            Set: Ethernet MAC address
            Enable: Obtain IP address automatically
    ```

* Enable startup from SD card.
    ```
    Image Packaging Configuration  --->
        Root filesystem type (INITRD)  --->
            Enable: INITRD
    ```

* Set hostname.
    ```
    Firmware Version Configuration  --->
        Set: Host name
        Set: Product name
        Set: Firmware version
    ```

### Rootfs configuration

Changes in the rootfs are required to configure some features. You can edit it using `petalinux-config -c rootfs` tool.

* Ensure dropbear SSH server is enabled.
    ```
    Image Features  --->
        Enable: ssh-server-dropbear
    ```

* Enable debug tweaks and auto-login to enable remote development.
    ```
    Image Features  --->
        Enable: debug-tweaks
        Enable: auto-login
    ```

### Kernel configuration

Some changes in kernel modules are required to enable some USB features. You can edit it using `petalinux-config -c kernel` tool.

**Changes:**

 - Enable `CONFIG_USB_SERIAL_PL2303` module to enable Prolific USB-UART converter.

 - Enable `USB_SERIAL_XSENS_MT` module to enable Xsens MTi USB-UART converter.

 - Enable `INPUT_JOYSTICK` and `INPUT_JOYDEV` modules to activate joystick support. 

### Robot Operating System (ROS2-Foxy) configuration

> This part is mostly based on the [ROS 2 Humble Hawksbill with Yocto and PetaLinux](https://news.accelerationrobotics.com/ros2-humble-yocto-petalinux/) article.

**Steps:**  

- Clone modified `meta-ros` package:
    
    ```bash
    git clone https://github.com/PUTvision/put-meta-ros $WORKSPACE/project-spec/meta-ros
    ```

- Edit configuration file: `$WORKSPACE/build/conf/bblayers.conf` and add parts:

    - add headers: 
        ```bash
        # define the ROS 2 Yocto target release
        ROS_OE_RELEASE_SERIES = "honister"

        # define ROS 2 distro
        ROS_DISTRO = "foxy"
        ```

    - append sources:
        ```bash
        ${SDKBASEMETAPATH}/../../project-spec/meta-ros/meta-ros2-foxy \
        ${SDKBASEMETAPATH}/../../project-spec/meta-ros/meta-ros2 \
        ${SDKBASEMETAPATH}/../../project-spec/meta-ros/meta-ros-common \
        ```

- Create destination image directory:  
    ```bash
        mkdir -p  $WORKSPACE/project-spec/meta-user/recipes-images/images
    ```
    and copy our configuration file:  
    ```bash
        cp $WORKSPACE/project-spec/meta-ros/put_sources/petalinux-image-minimal.bbappend $WORKSPACE/project-spec/meta-user/recipes-images/images/`
    ```

### Application configuration

* Each layer below clone into the `$WORKSPACE/project-spec/meta-user/recipes-apps/` directory.

* Our application layers:

    - [put-meta-base-config](https://github.com/PUTvision/put-meta-base-config) - Base configuration containing common things like bashrc, etc.
    - [put-meta-depthai-core](https://github.com/PUTvision/put-meta-depthai-core) - DepthAI core package. Based on [depthai-core](https://github.com/luxonis/depthai-core) sources. It enables the usage of DepthAI API in C++ and Python. 
    - [put-meta-depthai-ros](https://github.com/PUTvision/put-meta-depthai-ros) - ROS bindings for DepthAI. Based on [depthai-ros/foxy](https://github.com/luxonis/depthai-ros) sources. It provides ROS2 nodes for DepthAI API.
    - [put-meta-husky](https://github.com/PUTvision/put-meta-husky) - Husky mobile robot package for Petalinux usage. Based on [husky/foxy-devel](https://github.com/husky/husky/tree/foxy-devel) package. 
    - [put-meta-xsens](https://github.com/PUTvision/put-meta-xsens) - modified Xsens package. Based od [bluespace_ai_xsens_ros_mti_driver](https://github.com/bluespace-ai/bluespace_ai_xsens_ros_mti_driver).

### Onboard configuration (first boot)

* Connect via USB using serial console (115200 baud rate, 8N1, no flow control).
    ```bash
    sudo picocom -b 115200 /dev/ttyUSB1
    ```

    > **Note:** You have to use USB1 port.

    Then, set bash as default shell (.bashrc and .bash_profile are used by default)

    ```bash
    chsh -s /bin/bash root
    ```

    Call `bash` command to start bash shell. You should see welcome message, together with some information about the system: **ETHERNET IP ADDRESS** and **ROS_DOMAIN_ID**.

    After that, you can login via SSH using `root` user and `root` password.

* Create ROS2 inference engine node
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src

    git clone https://github.com/PUTvision/ros2_fpga_inference_node.git inference_node/
    cd ~/ros2_ws

    colcon build
    source install/setup.bash
    ```

    > **Note:** Note that actually the repository is private. You have to contact with the author to get access to it.

### Usage:

```bash
# run inference engine
ros2 run inference_node inference_engine 
```

> **Scripts below are added to Systemd services and you don't need to run it.**

```bash
# run imu node
ros2 launch bluespace_ai_xsens_mti_driver xsens_mti_node.launch.py

# run husky node
ros2 launch husky_base put_base.launch.py

# run camera node
ros2 launch depthai_cognition rgb_depth_publisher.launch.py
```

## Example inference

![rqt_viz](https://putvision.github.io/assets/images/posts/2023/02/husky_inf.webp)


## Known issues

* Booting process is performed 3 times. It is caused by the fact that the system is not able to find the rootfs partition.

* Sometimes, during a boot process, the system hangs with a message like: "No found devices in /dev...". In this case, you have to reboot the system. 

* When booting without Internet connection, it is required to set datatime manually. Otherwise, the ROS system will hang. Because we set `date -s "2023-01-01 12:00:00"` as systemd service, it is required to update the date after 2023.

* Boot time is very long. From switching on the power to begin of communication with the Husky robot, it takes about 2 minutes.

## Acknowledge

<img src="https://upload.wikimedia.org/wikipedia/commons/a/af/ESA_logo.png" height="60px" />

**This work was funded by European Space Agency OSIP, PO number: 4000138073, COGNITION project.**
