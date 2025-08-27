#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose2D
import time
import os
import select

GANTRY = "/home/jarvis/printer_data/comms/klippy.serial" # Klipper G-code I/O socket
SPEED = 500                       # Movement feedrate (mm/s)

# Track pump state so we don't spam SET_PIN unnecessarily
current_pump_state = 0  

def write_gcode(cmd: str):
    """Send G-code command to Klipper pipe."""
    try:
        with open(GANTRY, "w") as fifo:
            fifo.write(cmd + "\n")
            fifo.flush()
    except Exception as e:
        rospy.logerr(f"Pipe write failed: {e}")

def wait_for_ok(timeout=15):
    """Wait for 'ok' or 'home' response from Klipper after G28."""
    try:
        with open(GANTRY, "r") as printer:
            poller = select.poll()
            poller.register(printer, select.POLLIN)
            start = time.time()
            while time.time() - start < timeout:
                events = poller.poll(1000)
                if events:
                    line = printer.readline().strip()
                    rospy.loginfo(f"[Klipper] {line}")
                    if "ok" in line.lower() or "home" in line.lower():
                        return True
    except Exception as e:
        rospy.logerr(f"Klipper response wait failed: {e}")
    return False

def set_pump(state: int):
    """Turn pump ON (1) or OFF (0) only if state changes."""
    global current_pump_state
    if state != current_pump_state:
        cmd = f"SET_PIN PIN=pump VALUE={state}"
        rospy.loginfo(f"[PUMP] {'ON' if state else 'OFF'}")
        write_gcode(cmd)
        current_pump_state = state

def crack_callback(msg: Pose2D):
    """Convert crack detection data → Klipper G-code."""
    # Move
    gcode_move = f"G1 X{msg.x:.2f} Y{msg.y:.2f} F{SPEED*60}"
    rospy.loginfo(f"[MOVE] {gcode_move}")
    write_gcode(gcode_move)

    # Pump control synced with theta
    pump_state = 1 if msg.theta >= 1.0 else 0
    set_pump(pump_state)

def main():
    rospy.init_node("ros_to_klipper_stream", anonymous=True)

    # --- HOMING ---
    rospy.loginfo("Sending homing command...")
    set_pump(0)             # Ensure pump is off during homing
    write_gcode("G28")
    set_pump(pump_state)    # Restore pump state after homing
    if wait_for_ok():
        rospy.loginfo("Homing complete.")
    else:
        rospy.logwarn("Homing timeout — continuing anyway.")

    # --- Pump Sync ---
    # In case ROS already published a crack before we start listening
    initial_theta = rospy.get_param("~initial_theta", 0)
    set_pump(1 if initial_theta >= 1.0 else 0)

    # Subscriber
    rospy.Subscriber("/crack_coordinates", Pose2D, crack_callback)
    rospy.loginfo("ROS → Klipper bridge running...")
    rospy.spin()

if __name__ == "__main__":
    main()
