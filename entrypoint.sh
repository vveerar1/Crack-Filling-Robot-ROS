#!/bin/bash

set -e

# Optional: launch ROS file if specified
if [ "$1" == "launch" ]; then
    shift
    echo "🔄 Launching: $@"
    exec ros2 launch "$@"
else
    # Stay alive for dev shells or exec bash fallback
    echo "✅ Done creating container. You can now run ROS commands."
    echo "🔄 Starting bash shell..."
    exec $@
fi