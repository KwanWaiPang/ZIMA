[comment]: <> (# ZIMA)

<h1 align="center"> ZIMA (轻量模块化可移植的2D激光SLAM导航家用清洁机器人算法SDK)
</h1>

[comment]: <> ( <h2 align="center">PAPER</h2>)
  <h3 align="center">
  <a href="https://github.com/BitSoulLab/ZIMA" target="_blank">Original Github Page</a>
  | <a href="" target="_blank">Blog</a>
  </h3>
  <div align="center"></div>

<!-- rm -rf .git -->

# 配置

原本的配置四采用Docker镜像的
```bash
docker pull bitsoullab/ros:zima-dev
# Password for user zima is 123456

#容器创建启动方式
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

此处先不用docker，先进行编译安装
1. 在zima_base目录下，运行`bash build.sh build_dir`
2. 在zima_core目录下，运行`bash build.sh build_dir`

<div align="center">
  <img src="./Figs/2025-03-11 10-30-38 的屏幕截图.png" width="60%" />
<figcaption>  
</figcaption>
</div>

3. 新建一个ros workspace，然后使用catkin_make编译。

<div align="center">
  <img src="Figs/2025-03-11 10-33-39 的屏幕截图.png" width="60%" />
<figcaption>  
</figcaption>
</div>

运行前先确保已经source了工作空间
```bash
source /your_path_to_workspace/devel/setup.bash
```

<div align="center">
  <img src="Figs/2025-03-11 10-34-52 的屏幕截图.png" width="60%" />
<figcaption>  
</figcaption>
</div>


# 测试
1. 启动仿真环境
~~~
roslaunch zima_gazebo gazebo.launch
~~~

2. 启动demo
~~~
roslaunch zima_ros gazebo_demo.launch
~~~

3. 启动rviz
~~~
roslaunch zima_ros rviz.launch
~~~