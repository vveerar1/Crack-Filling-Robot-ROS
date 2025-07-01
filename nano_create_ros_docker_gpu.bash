# If not working, first do: sudo rm -rf /tmp/.docker.xauth
# It still not working, try running the script as root.
## Build the image first
### docker build -t r2_path_planning .
## then run this script
xhost local:root

XAUTH=/tmp/.docker.xauth

docker pull dustynv/ros:humble-desktop-l4t-r36.4.0

docker stop ros2

docker rm ros2

docker run -it \
    --name=ros2 \
    --network host \
    --runtime nvidia \
    --gpus all \
    --privileged \
    --restart=unless-stopped \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --volume="$PWD":"/home/ros":"rw" \
    --volume="$PWD/docker.bashrc":"/root/.bashrc":"ro" \
    dustynv/ros:humble-desktop-l4t-r36.4.0 \
    bash

echo "Done."