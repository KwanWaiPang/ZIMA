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
1. 在zima_base目录下，运行`sh build.sh`
2. 在zima_core目录下，运行`sh build.sh`
3. 新建一个ros workspace，导入zima_gazebo/zima_ros。并使用catkin_make编译。
