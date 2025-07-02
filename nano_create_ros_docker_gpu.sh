#!/bin/bash 

xhost local:root

XAUTH=/tmp/.docker.xauth

docker pull osrf/ros:humble-desktop-full-jammy

docker ps -a --format '{{.Names}}' | grep -qw ros2 && docker stop ros2 > /dev/null

docker ps -a --format '{{.Names}}' | grep -qw ros2 && docker rm ros2 > /dev/null

docker run -it \
    --name=ros2 \
    --network=host \
    --runtime=nvidia \
    --gpus=all \
    --privileged \
    --restart=unless-stopped \
    --entrypoint=/entrypoint.sh \
    --volume="$PWD/entrypoint_pc.sh":/entrypoint.sh \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --volume="$PWD/ROS2/ros2_ws":"/home/ros/ros2_ws":"rw" \
    dustynv/ros:humble-desktop-l4t-r36.4.0 \

