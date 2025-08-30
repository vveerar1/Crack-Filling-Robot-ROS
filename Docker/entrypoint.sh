#!/usr/bin/env bash

set -e

# Start the GUI stack if installed
if $INSTALL_VNC ; then
    if /usr/local/share/desktop-init.sh; then
        echo "GUI stack started successfully"
    else
        echo "Error: GUI stack failed to start" >&2
        exit 1
    fi
else
    echo "GUI stack not installed, skipping..."
fi

# Initialize Arduino CLI
echo "Initializing Arduino-Cli"
# arduino-cli board list
arduino-cli core install $(arduino-cli board list | awk '!/Unknown/ && NR>1 {print $NF}' | sort -u)
if [ ! -d "/home/ros/Arduino/libraries/Sabertooth" ]; then
    arduino-cli config set library.enable_unsafe_install true
    arduino-cli lib install --zip-path /home/ros/Arduino/Sabertooth.zip
fi

# Optional: launch ROS file if specified
if [ "$1" == "launch" ]; then
    shift
    echo "Launching ROS: $@"
    exec ros2 launch "$@"
elif [ $# -eq 0 ]; then
    echo "Starting interactive shell..."
    exec bash
else
    echo "Executing entry command: $@"
    exec "$@"
fi