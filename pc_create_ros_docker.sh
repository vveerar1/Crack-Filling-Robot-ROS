#!/bin/bash 

xhost local:root

XAUTH=/tmp/.docker.xauth

docker pull osrf/ros:humble-desktop-full-jammy

docker ps -a --format '{{.Names}}' | grep -qw ros2 && docker stop ros2 > /dev/null

docker ps -a --format '{{.Names}}' | grep -qw ros2 && docker rm ros2 > /dev/null

# Append these to the docker run commant if using NVIDIA GPU
# If you want to use the NVIDIA GPU, make sure to have the NVIDIA Container Toolkit installed.

    # --runtime=nvidia \
    # --gpus=all \

# If passing a USB device, example Arduino
    # --device=/dev/ttyACM0 \   

# If passing an input device, example keyboard or mouse or joystick
# For use access permissions, we have to use --privileged
    # --device=/dev/input \ 
    # --privileged \

docker run -it \
    --name=ros2 \
    --network=host \
    --restart=unless-stopped \
    --entrypoint=/entrypoint.sh \
    --volume="$PWD/entrypoint_pc.sh":/entrypoint.sh \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --volume="$PWD/ROS2/ros2_pc":"/home/ros/ros2_ws":"rw" \
    osrf/ros:humble-desktop-full-jammy \
