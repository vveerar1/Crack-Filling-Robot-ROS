#!/bin/bash

# If you're running GUI tools like rviz2, the entrypoint can also check for display 
# connectivity or even launch a ROS node directly.

set -e

# 👤 Optional: create ros user and configure if not already done
id -u ros &>/dev/null || {
    adduser --disabled-password --gecos "" --home /home/ros --no-create-home ros 1>/dev/null
    apt update && apt install -y ros-humble-rmw-cyclonedds-cpp
    cp /etc/skel/.bash_logout /etc/skel/.bashrc /etc/skel/.profile /home/ros/
    touch /home/ros/.inputrc
    touch /etc/udev/rules.d/99-input.rules
    echo 'KERNEL=="event*", SUBSYSTEM=="input", GROUP="input", MODE="0660"' >> /etc/udev/rules.d/99-input.rules
    echo 'KERNEL=="js*", SUBSYSTEM=="input", GROUP="input", MODE="0660"' >> /etc/udev/rules.d/99-input.rules
    chown -R ros:ros /home/ros
    echo 'ros ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers.d/ros
    chmod 0440 /etc/sudoers.d/ros
    echo '# Respect default shortcuts.' >> /home/ros/.inputrc
    echo '$include /etc/inputrc' >> /home/ros/.inputrc
    echo -e '\n## arrow up' >> /home/ros/.inputrc
    echo '"\e[A":history-search-backward' >> /home/ros/.inputrc
    echo '## arrow down' >> /home/ros/.inputrc
    echo '"\e[B":history-search-forward' >> /home/ros/.inputrc
    echo -e "\n# Display Passthrough Variables" >> /home/ros/.bashrc
    echo -e "export DISPLAY=$DISPLAY" >> /home/ros/.bashrc
    echo -e "export QT_X11_NO_MITSHM=1" >> /home/ros/.bashrc
    echo -e "export XAUTHORITY=$XAUTHORITY" >> /home/ros/.bashrc
    echo -e "\n# ROS 2 Variables" >> /home/ros/.bashrc
    echo -e "export ROS_DOMAIN_ID=0" >> /home/ros/.bashrc
    echo -e "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/ros/.bashrc
    echo -e "unset ROS_LOCALHOST_ONLY" >> /home/ros/.bashrc
    echo -e "source /opt/ros/humble/setup.bash" >> /home/ros/.bashrc
    echo -e "source /home/ros/ros2_ws/install/setup.bash" >> /home/ros/.bashrc
    usermod -aG dialout ros
    for dev in /dev/ttyACM*; do sudo chmod a+rw "$dev"; done
}

# 🌐 Set environment variables
export DISPLAY=${DISPLAY}
export QT_X11_NO_MITSHM=1
export XAUTHORITY=${XAUTHORITY}
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_LOCALHOST_ONLY

# 📦 Source ROS setup
source /opt/ros/humble/setup.bash
source /home/ros/ros2_ws/install/setup.bash

# 🛠️ Run udev rules to ensure input devices are recognized
/lib/systemd/systemd-udevd --daemon
udevadm control --reload-rules
udevadm trigger

# 🚀 Optional: launch ROS file if specified
if [ "$1" == "launch" ]; then
    shift
    echo "🔄 Launching: $@"
    exec ros2 launch "$@"
else
    # 🧘 Stay alive for dev shells or exec bash fallback
    echo "✅ Done creating container. You can now run ROS commands."
    echo "🔄 Starting bash shell..."
    su - ros
fi
