# Zima SDK

![License](https://img.shields.io/badge/License-MIT-blue.svg)

[项目地址](https://github.com/BitSoulLab/ZIMA.git)

Zima SDK是一个力求轻量模块化可移植的2D激光SLAM导航家用清洁机器人算法SDK。

**目前已使用MIT协议开源。**

SDK包含基础数据格式、控制算法、规划算法、SLAM算法，低依赖（目前只需glog/gflags/protobuf）。为便于调试，也加入了ros封装和简单gazebo仿真。
目前Gazebo仿真Demo部署方式为docker。(Demo已内置里程计的累计误差模拟，和雷达的测量误差模拟，但雷达的运动畸变模拟并未加入)

## Installation

（示例宿主机为Ubuntu22.04系统，理论上可兼容其他Linux发行版）：

1. 宿主机需要先安装Docker，并拉取镜像。

    ```bash
    docker pull bitsoullab/ros:zima-dev
    # Password for user zima is 123456
    ```

    容器创建启动方式：

    ```bash
    if [ -e /dev/nvidia0 ]; then
      echo "Launch with nvidia support."
      docker run \
        -it \
        -u zima \
        --name="zima_demo" \
        --net=host \
        --privileged \
        -v /dev:/dev \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        --runtime=nvidia \
        --device /dev/nvidia0 \
        --device /dev/nvidia-uvm \
        --device /dev/nvidia-uvm-tools \
        --device /dev/nvidiactl \
        --runtime=nvidia \
        --gpus all \
        bitsoullab/ros:zima-dev
    else
      echo "Launch without nvidia support."
      docker run \
        -it \
        -u zima \
        --name="zima_demo" \
        --net=host \
        --privileged \
        -v /dev:/dev \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        bitsoullab/ros:zima-dev
    fi
    ```

1. 编译安装
    1. 在zima_base目录下，运行`build.sh`。
    1. 在zima_core目录下，运行`build.sh`。
    1. 新建一个ros workspace，导入zima_gazebo/zima_ros。并使用`catkin_make`编译。

1. 仿真测试

    运行ros命令前请记得添加zima包的环境变量。

    ```bash
    source /your/path/to/workspace/devel/setup.bash
    ```

    容器中启动仿真环境方法(在独立终端中运行)：

    ```bash
    roslaunch zima_gazebo gazebo.launch
    ```

    容器中启动Demo(在独立终端中运行)：

    ```bash
    roslaunch zima_ros gazebo_demo.launch
    ```

    若遇到报错 `Open: Open /tmp/zima_config.json failed.`，请在zima_base/json_config中复制一份zima_dev_gazebo_config.json到/tmp/zima_config.json。

    容器中启动Rviz(在独立终端中运行)：

    ```bash
    roslaunch zima_ros rviz.launch
    ```

    建议仿真环境与Demo与Rviz从不同的终端窗口进入docker后启动，因为Demo程序使用键盘标准输入为测试命令输入，用一个launch文件一起启动的话键盘输入会失效。
    键盘控制详细请看Demo程序输出提示，若提示日志已被刷走，可按esc键或任意非功能键来输出提示。

## 适配硬件

此项目已适配Kobuki底盘，但需要自行搭配arm linux核心板以及雷达使用。

## 欢迎订阅

微信公众号/CSDN/知乎/小红书/BiliBili：比特有灵
Github: BitSoulLab
