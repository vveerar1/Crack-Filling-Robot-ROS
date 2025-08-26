#!/usr/bin/env bash

set -e

# Start supervisord in background
# if supervisord -c /etc/supervisor/supervisord.conf >/dev/null 2>&1; then
#     echo "✅ GUI stack started successfully"
# else
#     echo "❌ GUI stack failed to start"
# fi
if $INSTALL_VNC ; then
    echo "GUI stack started successfully"
    /usr/local/share/desktop-init.sh
else
    echo "GUI stack failed to start"
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