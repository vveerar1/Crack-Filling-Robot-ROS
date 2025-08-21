
# PATH for Python packages
export PATH=$PATH:/home/ros/.local/bin

# Display Passthrough Variables
export DISPLAY=$DISPLAY
# export QT_X11_NO_MITSHM=1
# export XAUTHORITY=$XAUTHORITY

# ROS 2 Variables
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_LOCALHOST_ONLY
source /home/ros/ros2_ws/install/setup.bash
