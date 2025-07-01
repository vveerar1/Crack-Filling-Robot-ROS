# If not working, first do: sudo rm -rf /tmp/.docker.xauth
# It still not working, try running the script as root.
## Build the image first
### docker build -t r2_path_planning .
## then run this script

docker start ros2 && \

docker exec -it --user 1000:1000 \
    -w /home/ros/ ros2 bash