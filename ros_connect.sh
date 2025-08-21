#!/bin/bash 

docker ps -a --format '{{.Names}}' | grep -qw ros2 && docker start ros2 > /dev/null

# docker exec -it --user 1000:1000 \
#     -w /home/ros/ ros2 bash

docker exec -it --user ros ros2 bash #-c "sudo /lib/systemd/systemd-udevd --daemon && sudo udevadm trigger && exec bash --login"
