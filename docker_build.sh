#!/bin/bash 

ARCH=$(uname -m)

build_image() {
  if [ "$ARCH" = "aarch64" ]; then
    IMAGE="dustynv/ros:humble-desktop-l4t-r36.4.0"
  else
    IMAGE="osrf/ros:humble-desktop-full-jammy"
  fi

  docker build --build-arg ROS_IMAGE=$IMAGE -t ros2 .
}

run_container() {
  if [ "$ARCH" = "aarch64" ]; then
    WORKSPS="ros2_nano"
  else
    WORKSPS="ros2_pc"
  fi

  xhost local:root

  docker ps -a --format '{{.Names}}' | grep -qw ros2 && docker stop ros2 > /dev/null

  docker ps -a --format '{{.Names}}' | grep -qw ros2 && docker rm ros2 > /dev/null

  # Append these to the docker run commant if using NVIDIA GPU
  # If you want to use the NVIDIA GPU, make sure to have the NVIDIA Container Toolkit installed.

      # --runtime=nvidia \
      # --gpus=all \
      # --env="QT_X11_NO_MITSHM=1"
      # --env="NVIDIA_VISIBLE_DEVICES=all" \  
      # --env="NVIDIA_DRIVER_CAPABILITIES=all" \

  # If passing a USB device, example Arduino
      # --device=/dev/ttyACM0 \   

  # If passing an input device, example keyboard or mouse or joystick
  # For use access permissions, we have to use --privileged
      # --device=/dev/input \ 
  # For more flexibilty and hotplugging options we can use c group rules
      # --volume="/dev/input:/dev/input" \
      # --device-cgroup-rule="c 13:* rmw" \
  # Not recommended, but if you know what you're doing, you can run the container 
  # in privileged mode to give the container complete access to the host system.
      # --privileged \ 

  docker run -it \
      --user=ros \
      --name=ros2 \
      --network=host \
      --ipc=host \
      --volume="/dev:/dev" \
      --device-cgroup-rule="c 13:* rmw" \
      --device-cgroup-rule="c 189:* rmw" \
      --device-cgroup-rule="c 166:* rmw" \
      --env="DISPLAY=:0" \
      --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
      --volume="$PWD/ROS2/$WORKSPS":"/home/ros/ros2_ws":"rw" \
      ros2
}

# Parse flags
case "$1" in
  -b)
    build_image
    run_container
    ;;
  -r)
    run_container
    ;;
  *)
    echo "Usage: $0 [-b | -r]"
    echo "  -b  Build and run"
    echo "  -r  Run only"
    exit 1
    ;;
esac
