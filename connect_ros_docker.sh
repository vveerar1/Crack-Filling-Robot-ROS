#!/bin/bash 

docker ps -a --format '{{.Names}}' | grep -qw ros2 && docker start ros2 > /dev/null

# docker exec -it --user 1000:1000 \
#     -w /home/ros/ ros2 bash

docker exec -it ros2 su - ros
