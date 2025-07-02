#!/bin/bash 

xhost local:root

XAUTH=/tmp/.docker.xauth

docker pull osrf/ros:humble-desktop-full-jammy

docker ps -a --format '{{.Names}}' | grep -qw ros2 && docker stop ros2 > /dev/null

docker ps -a --format '{{.Names}}' | grep -qw ros2 && docker rm ros2 > /dev/null

docker run -it \
    --name=ros2 \
    --network host \
    --privileged \
    --restart=unless-stopped \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --volume="$PWD/ROS2/ros2_ws":"/home/ros/ros2_ws":"rw" \
    osrf/ros:humble-desktop-full-jammy \
    bash -c "id -u ros &>/dev/null || (adduser --disabled-password --gecos '' --home /home/ros --no-create-home ros && chown -R ros:ros /home/ros && apt update \
    && apt install -y ros-humble-rmw-cyclonedds-cpp && echo -e \"\nros ALL=(ALL) NOPASSWD:ALL\" >> /etc/sudoers.d/ros);\
    su - ros -c 'cp /etc/skel/.bash_logout /etc/skel/.bashrc /etc/skel/.profile /home/ros/\
    && echo -e \"\nexport DISPLAY=$DISPLAY\nexport QT_X11_NO_MITSHM=1\nexport XAUTHORITY=$XAUTHORITY\
    \n\nexport ROS_DOMAIN_ID=0\nexport RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\
    \nunset ROS_LOCALHOST_ONLY\nsource /opt/ros/humble/setup.bash\nsource /home/ros/ros2_ws/install/setup.bash\" >> ~/.bashrc\
    && source ~/.bashrc'; su - ros -c 'exec bash'"

echo "Done."
