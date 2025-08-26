#!/bin/bash 

docker ps -a --format '{{.Names}}' | grep -qw ros2 && docker start ros2 > /dev/null

docker exec -it --user ros ros2 bash 
