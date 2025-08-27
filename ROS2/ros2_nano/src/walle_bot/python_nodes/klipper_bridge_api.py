#!/usr/bin/env python3
import rospy
import requests
from geometry_msgs.msg import Pose2D

# Config
KLIPPER_HOST = "http://<klipper_host>:7125"  # Replace with Jetson IP or localhost

def send_gcode(cmd: str):
    """Send raw G-code to Klipper via Moonraker"""
    try:
        r = requests.post(f"{KLIPPER_HOST}/printer/gcode/script",
                          json={"script": cmd}, timeout=2)
        if r.status_code != 200:
            rospy.logwarn(f"Klipper rejected command: {cmd}, status={r.status_code}")
    except Exception as e:
        rospy.logerr(f"Error sending gcode: {e}")

def crack_callback(msg: Pose2D):
    """
    ROS callback for crack coordinates.
    msg.x, msg.y = target position (mm)
    msg.theta = pump state (1=ON, 0=OFF)
    """

    # Move to XY location
    gcode_move = f"G1 X{msg.x:.2f} Y{msg.y:.2f} F6000"
    rospy.loginfo(f"[MOVE] {gcode_move}")
    send_gcode(gcode_move)

    # Pump control from theta
    if msg.theta >= 1.0:
        rospy.loginfo("[PUMP] ON")
        send_gcode("SET_PIN PIN=pump VALUE=1")
    else:
        rospy.loginfo("[PUMP] OFF")
        send_gcode("SET_PIN PIN=pump VALUE=0")

def main():
    rospy.init_node("klipper_bridge", anonymous=True)

    # Subscribe to crack detector topic
    rospy.Subscriber("/crack_coordinates", Pose2D, crack_callback)

    rospy.loginfo("Klipper bridge node started. Waiting for crack data...")
    rospy.spin()

if __name__ == "__main__":
    main()

# End of file
# How to Use
# In your ROS workspace (catkin_ws or colcon_ws), create a new package crack_filler_bridge.

# Add klipper_bridge.py to scripts/ and make it executable:

    # chmod +x klipper_bridge.py

# Run with:

    # rosrun crack_filler_bridge klipper_bridge.py

# If your crack detector publishes like this:

    # rostopic pub /crack_coordinates geometry_msgs/Pose2D '{x: 120.0, y: 45.0, theta: 1.0}'

# Robot moves to (120,45) and turns pump ON.

    # rostopic pub /crack_coordinates geometry_msgs/Pose2D '{x: 130.0, y: 45.0, theta: 0.0}'

# Robot moves to (130,45) and turns pump OFF.