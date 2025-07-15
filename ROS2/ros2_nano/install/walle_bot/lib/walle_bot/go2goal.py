#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import numpy as np 

class Go2GoalNode(Node):
    """
    ROS2 node for navigating a robot to a goal position using PID control.
    Publishes velocity commands based on the current position and orientation.
    """

    def __init__(self):
        super().__init__('go2goal_node')

        # Declare parameters with default values
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_theta', 0.0)
        self.declare_parameter('kp_linear', 1.0)
        self.declare_parameter('kp_angular', 1.0)
        self.declare_parameter('max_linear_speed', 2.0)  # m/s
        self.declare_parameter('max_angular_speed', 2.0)  # rad/s
        self.declare_parameter('goal_tolerance', 0.1)    # meters
        self.declare_parameter('angle_tolerance', 0.01)   # radians

        # Get parameter values
        self.goal_x = self.get_parameter('goal_x').get_parameter_value().double_value
        self.goal_y = self.get_parameter('goal_y').get_parameter_value().double_value
        self.goal_theta = self.get_parameter('goal_theta').get_parameter_value().double_value
        self.kp_linear = self.get_parameter('kp_linear').get_parameter_value().double_value
        self.kp_angular = self.get_parameter('kp_angular').get_parameter_value().double_value
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        self.angle_tolerance = self.get_parameter('angle_tolerance').get_parameter_value().double_value

        # ROS publishers/subscribers
        self.turtle_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Pose, '/turtle1/pose', self.current_pose_callback, 10)

        # Timer for periodic updates
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz

        # Initialize state variables
        self.pose = None
        self.rot_done = False

    def current_pose_callback(self, msg):
        """Callback to update current position and orientation."""
        self.pose = msg

    def timer_callback(self):
        """Timer callback to compute and publish velocity commands."""
        # Calculate errors
        distance_to_goal = math.sqrt((self.goal_x - self.pose.x) ** 2 + (self.goal_y - self.pose.y) ** 2) # For TurtleSim
        dist_to_goal_x = self.goal_x - self.pose.x # For WallE Mecannum Drive
        dist_to_goal_y = self.goal_y - self.pose.y # For WallE Mecannum Drive
        angle_to_goal = math.atan2(self.goal_y - self.pose.y, self.goal_x - self.pose.x)
        angle_error = angle_to_goal - self.pose.theta
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]
            
        # Calculate velocities with safety limits
        linear_velocity = min(self.kp_linear * distance_to_goal, self.max_linear_speed)
        angular_velocity = min(self.kp_angular * angle_error, self.max_angular_speed)
        # linear_x_vel = min(self.kp_linear * dist_to_goal_x, self.max_linear_speed)
        # linear_y_vel = min(self.kp_linear * dist_to_goal_y, self.max_linear_speed)
        raw_vector = np.array([dist_to_goal_x, dist_to_goal_y])
        norm = np.linalg.norm(raw_vector)
        if norm > 1e-3:
            scaled_vector = raw_vector * min(self.kp_linear * norm, self.max_linear_speed) / norm
            linear_x_vel, linear_y_vel = scaled_vector
        
        # Slow down when turning sharply
        if abs(angle_error) > math.pi/4:
            linear_velocity *= 0.5
        
        # Create and publish command    
        new_turt_vel = Twist()
        new_vel = Twist()
        
        if self.rot_done:
            angle_error = 0.0  # Reset angle error if rotation is done
            
        # Check if we reached the goal
        if abs(angle_error) > self.angle_tolerance:
            new_turt_vel.angular.z = angular_velocity
        elif distance_to_goal > self.goal_tolerance:
            self.rot_done = True # Assume rotation is done if within angle tolerance
            new_turt_vel.linear.x = linear_velocity
            new_vel.linear.x = linear_x_vel
            new_vel.linear.y = linear_y_vel
        else:
            # Stop the robot if within goal tolerance
            self.stop_robot()
            self.get_logger().info("Goal reached!")
            return    

        self.turtle_vel_pub.publish(new_turt_vel)
        self.cmd_vel_pub.publish(new_vel)
        
        # Log status
        self.get_logger().info(f"Distance: {distance_to_goal:.2f}m, Angle error: {math.degrees(angle_error):.1f}°")

    def stop_robot(self):
        """Send zero velocity command to stop the robot."""
        stop_vel = Twist()
        stop_vel.linear.x = 0.0
        stop_vel.linear.y = 0.0
        stop_vel.angular.z = 0.0
        self.turtle_vel_pub.publish(stop_vel)
        self.cmd_vel_pub.publish(stop_vel)
        
    def destroy_node(self):
        """Clean up resources before shutting down."""
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Go2GoalNode()
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