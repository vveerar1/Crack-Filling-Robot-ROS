#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import serial

class ArduinoNode(Node):
    def __init__(self):
        super().__init__('arduino_node')
        self.odom_pub = self.create_publisher(String, 'arduino_odometer', 10)
        self.cmd_pub = self.create_publisher(String, 'arduino_cmd_vel', 10)
        # self.subscription = self.create_subscription(
        #     String,
        #     'arduino_cmd',
        #     self.arduino_cmd_callback,
        #     10)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Declare parameters with default values
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 9600)

        # Get values (overridden if passed via launch or YAML)
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        
        self.get_logger().info(f"🛠 Using serial port: {serial_port} at {baud_rate} baud")
        
        try:
            self.arduino = serial.Serial(serial_port, baud_rate, timeout=0.01)
            self.get_logger().info("✅ Serial connection established")
        except serial.SerialException as e:
            self.get_logger().error(f'❌ Could not open serial port: {e}')
            raise e
        
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz

        # Robot parameters (adjust as needed)
        self.L = 0.18  # Half of robot length (meters)
        self.W = 0.15  # Half of robot width (meters)
        self.R = 0.03  # Wheel radius (meters)
        self.max_speed = 255  # Max motor speed

    def timer_callback(self):
        try:
            if self.arduino.in_waiting:
                data = self.arduino.readline().decode('utf-8').strip()
                if data:
                    msg = String()
                    msg.data = data
                    self.odom_pub.publish(msg)
                    self.get_logger().debug(f'Published odometer: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error reading from Arduino: {e}')

    # def arduino_cmd_callback(self, msg):
    #     try:
    #         self.arduino.write((msg.data + '\n').encode('utf-8'))
    #         self.get_logger().info(f'Sent to Arduino: {msg.data}')
    #     except Exception as e:
    #         self.get_logger().error(f'Error writing to Arduino: {e}')

    def cmd_vel_callback(self, msg):
        # Mecanum inverse kinematics
        Vx = msg.linear.x      # Forward/backward
        Vy = msg.linear.y      # Left/right
        Wz = msg.angular.z     # Rotation

        L = self.L
        W = self.W
        R = self.R

        # Calculate wheel speeds (in m/s)
        v1 = (1/R) * (Vx + Vy + (L + W) * Wz)  # Front Right
        v2 = (1/R) * (Vx - Vy - (L + W) * Wz)  # Front Left
        v3 = (1/R) * (Vx + Vy - (L + W) * Wz)  # Rear Left
        v4 = (1/R) * (Vx - Vy + (L + W) * Wz)  # Rear Right

        # Map wheel speeds to motor commands (-255 to 255)
        speeds = [v1, v2, v3, v4]
        max_wheel_speed = max(abs(s) for s in speeds)
        if max_wheel_speed > 1.0:
            speeds = [s / max_wheel_speed for s in speeds]  # Normalize

        motor_speeds = [int(self.max_speed * s) for s in speeds]

        # Send to Arduino as "s1 s2 s3 s4"
        cmd_str = f"{motor_speeds[0]} {motor_speeds[1]} {motor_speeds[2]} {motor_speeds[3]}"
        try:
            self.arduino.write((cmd_str + '\n').encode('utf-8'))
            self.cmd_pub.publish(String(data=cmd_str))
            self.get_logger().info(f'Sent to Arduino (from cmd_vel): {cmd_str}')
        except Exception as e:
            self.get_logger().error(f'Error writing to Arduino: {e}')

    def destroy_node(self):
        if self.arduino.is_open:
            self.arduino.close()
        super().destroy_node()      

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()