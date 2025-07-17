#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import serial
import math

class ArduinoNode(Node):
    """
    ROS2 node for interfacing with Arduino for mecanum drive robot.
    Publishes odometry, subscribes to cmd_vel, and sends motor commands via serial.
    """

    def __init__(self):
        super().__init__('arduino_node')

        # Declare parameters with default values
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 9600)   # Serial baud rate
        self.declare_parameter('L', 0.1016)         # Half robot length (meters)
        self.declare_parameter('W', 0.1016)         # Half robot width (meters)
        self.declare_parameter('R', 0.0485)         # Wheel radius (meters)
        self.declare_parameter('max_speed', 255)    # Max motor speed (PWM)
        self.declare_parameter('min_speed', 100)    # Min motor speed (PWM)

        # Get parameter values
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.L = self.get_parameter('L').get_parameter_value().double_value
        self.W = self.get_parameter('W').get_parameter_value().double_value
        self.R = self.get_parameter('R').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().integer_value
        self.min_speed = self.get_parameter('min_speed').get_parameter_value().integer_value
        
        self.get_logger().info(f"🛠 Using serial port: {serial_port} at {baud_rate} baud")
        self.get_logger().info(f"Using L={self.L}, W={self.W}, R={self.R}, max_speed={self.max_speed}")
        
        try:
            self.arduino = serial.Serial(serial_port, baud_rate, timeout=0.01)
            self.get_logger().info("✅ Serial connection established")
        except serial.SerialException as e:
            self.get_logger().error(f'❌ Could not open serial port: {e}')
            raise e

        # ROS publishers/subscribers
        self.odom_pub = self.create_publisher(String, 'arduino_odometer', 10)
        self.cmd_pub = self.create_publisher(String, 'arduino_cmd_vel', 10)
        qos_profile = rclpy.qos.qos_profile_sensor_data
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, qos_profile)

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz
        
        # Add command monitoring and buffer management
        self.cmd_count = 0
        self.last_reset_count = 0
        self.reset_threshold = 100  # Reset connection after this many commands
        self.declare_parameter('buffer_size', 64)
        self.buffer_size = self.get_parameter('buffer_size').get_parameter_value().integer_value
        self.last_cmd_time = self.get_clock().now()
        self.min_cmd_interval = 1.0 / 20.0  # Minimum 50ms between commands
        
        # Add monitoring timer
        self.monitor_timer = self.create_timer(1.0, self.monitor_connection)
        
    def timer_callback(self):
        """Read odometry from Arduino and publish."""
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
            
    def monitor_connection(self):
        """Monitor connection health and reset if needed"""
        if self.cmd_count - self.last_reset_count > self.reset_threshold:
            # self.get_logger().warn(f"Command threshold reached ({self.cmd_count}), resetting connection")
            self.reset_serial()
            
    def reset_serial(self):
        """Reset the serial connection"""
        try:
            port = self.arduino.port
            baud = self.arduino.baudrate
            if self.arduino.is_open:
                self.arduino.close()
            self.arduino = serial.Serial(port, baud, timeout=0.01)
            self.last_reset_count = self.cmd_count
            self.get_logger().info("Serial connection reset successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to reset connection: {e}")
            
    def map_velocity_to_pwm(self, pwm_input: int) -> int:
        if pwm_input < 0:
            # Map -255 to 0 → -255 to -100
            return int((pwm_input + self.max_speed) * ((self.max_speed-self.min_speed) / self.max_speed) - self.max_speed)
        elif pwm_input > 0:
            # Map 0 to 255 → 100 to 255
            return int(pwm_input * ((self.max_speed-self.min_speed) / self.max_speed) + self.min_speed)
        else:
            return 0


    def cmd_vel_callback(self, msg: Twist):
        """Convert cmd_vel to wheel speeds and send to Arduino."""
         # Rate limiting
        now = self.get_clock().now()
        if (now - self.last_cmd_time).nanoseconds / 1e9 < self.min_cmd_interval:
            return
        self.last_cmd_time = now
        
        # Velocity Calculations
        Vx = msg.linear.x
        Vy = msg.linear.y
        Wz = msg.angular.z
        max_velocity = 2.0  # Max linear velocity in m/s

        L = self.L
        W = self.W
        R = self.R

        # Mecanum inverse kinematics (wheel speeds in m/s)
        # v1 = (1/R) * (Vx + Vy + (L + W) * Wz)
        # v2 = (1/R) * (Vx - Vy - (L + W) * Wz)
        # v3 = (1/R) * (Vx + Vy - (L + W) * Wz)
        # v4 = (1/R) * (Vx - Vy + (L + W) * Wz)
        # Mecanum inverse kinematics Without Rotation (wheel speeds in m/s) 
        # v1 = (1/R) * (Vx + Vy)  # Front-Left
        # v2 = (1/R) * (Vx - Vy)  # Front-Right
        # v3 = (1/R) * (Vx + Vy)  # Rear-Left
        # v4 = (1/R) * (Vx - Vy)  # Rear-Right
        # speeds = [v1, v2, v3, v4]
        
        # self.get_logger().info(f"Raw motor speeds (pre-scale): {speeds}")
        
        # Find the max possible for each component (for scaling)
        max_linear = (1/R) * (max_velocity + max_velocity)  # Vx=2, Vy=2
        max_angular = (1/R) * (L + W) * max_velocity # Wz=2

        # Scale so full linear or full rotation gives full speed
        scale_linear = self.max_speed / (max_linear / 2) 
        scale_angular = self.max_speed / max_angular

        # def scale_wheel(Vx, Vy, Wz, sign_vx, sign_vy, sign_wz):
        #     linear = (1/R) * (sign_vx * Vx + sign_vy * Vy) * scale_linear
        #     rot = (1/R) * (L + W) * sign_wz * Wz * scale_angular
        #     return int(linear + rot)
        
        def scale_wheel(Vx, Vy, Wz, sign_vx, sign_vy, sign_wz):
            linear_pwm = (1/R) * (sign_vx * Vx + sign_vy * Vy) * scale_linear
            rot_pwm = (1/R) * (L + W) * sign_wz * Wz * scale_angular

            # Apply separate PWM mapping
            total_pwm = self.map_velocity_to_pwm(linear_pwm + rot_pwm)
            return total_pwm

        motor_speeds = [
            scale_wheel(Vx, Vy, Wz, 1, 1, 1),   # v1
            scale_wheel(Vx, Vy, Wz, 1, -1, -1), # v2
            scale_wheel(Vx, Vy, Wz, 1, 1, -1),  # v3
            scale_wheel(Vx, Vy, Wz, 1, -1, 1),  # v4
        ]
        
        self.get_logger().info(f"Scaled Motor speeds (post-scaling): {motor_speeds}")

        # Clip to [-max_speed, max_speed]
        motor_speeds = [max(-self.max_speed, min(self.max_speed, ms)) for ms in motor_speeds]
        
        # # Ensure minimum and maximum speed is respected
        # motor_speeds = [max(-self.max_speed, min(self.max_speed, (self.min_speed if ms > 0 else -self.min_speed)
        #                                          if 0 < abs(ms) < self.min_speed else ms)) for ms in motor_speeds]

        # Send to Arduino as "s1 s2 s3 s4"
        cmd_str = f"{motor_speeds[0]} {motor_speeds[1]} {motor_speeds[2]} {motor_speeds[3]}"
        try:
            # Check buffer state
            if self.arduino.out_waiting > (self.buffer_size // 2):
                self.arduino.reset_output_buffer()
                self.get_logger().warn("Output buffer more than half full - clearing")
                
            cmd_bytes = (cmd_str + '\n').encode('utf-8')
            if len(cmd_bytes) > (self.buffer_size - self.arduino.out_waiting):
                self.arduino.reset_output_buffer()
                
            self.arduino.write(cmd_bytes)
            self.arduino.flush()
            self.cmd_count += 1
            self.cmd_pub.publish(String(data=cmd_str))
            
            # # Reduce logging frequency
            # if self.cmd_count % 50 == 0:
            #     self.cmd_pub.publish(String(data=cmd_str))
            #     self.get_logger().info(f'Command count: {self.cmd_count}, Buffer: {self.arduino.out_waiting}')
                
        except Exception as e:
            self.get_logger().error(f'Error writing to Arduino: {e}')
            self.reset_serial()

    def destroy_node(self):
        """Clean up serial connection on shutdown."""
        if hasattr(self, 'arduino') and self.arduino.is_open:
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
# End of file