#! /usr/bin/env python3
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
import math


class ObstacleDetection(Node):
    """
    Simple obstacle detection node that stops the robot when obstacles are too close.
    Uses a circular detection zone around the robot.
    
    TODO: Implement the detect_obstacle method to avoid obstacles!
    """
    def __init__(self):
        super().__init__("obstacle_detection")
        
        # Safety parameters - use ROS parameter
        self.declare_parameter("stop_distance", 0.35)  # Default if not specified
        self.stop_distance = self.get_parameter("stop_distance").get_parameter_value().double_value
        self.get_logger().info(f"Using stop_distance: {self.stop_distance}m")
        
        # Store received data
        self.scan_ranges = []
        self.has_scan_received = False
        
        # Default motion command (slow forward)
        self.tele_twist = Twist()
        self.tele_twist.linear.x = 0.2
        self.tele_twist.angular.z = 0.0

        # Set up quality of service
        qos = QoSProfile(depth=10)

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", qos)
        
        # Subscribe to laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan, "scan", self.scan_callback, qos_profile=qos_profile_sensor_data
        )

        # Subscribe to teleop commands
        self.cmd_vel_raw_sub = self.create_subscription(
            Twist, "cmd_vel_raw", self.cmd_vel_raw_callback, 
            qos_profile=qos_profile_sensor_data
        )

        # Set up timer for regular checking
        self.timer = self.create_timer(0.1, self.timer_callback)

    def scan_callback(self, msg):
        """Store laser scan data when received"""
        self.scan_ranges = msg.ranges
        self.has_scan_received = True

    def cmd_vel_raw_callback(self, msg):
        """Store teleop commands when received"""
        self.tele_twist = msg

    def timer_callback(self):
        """Regular function to check for obstacles"""
        if self.has_scan_received:
            self.detect_obstacle()

    def detect_obstacle(self):
        """
        TODO: Implement obstacle detection and avoidance!
        
        MAIN TASK:
        - Detect if any obstacle is too close to the robot (closer than self.stop_distance)
        - If an obstacle is detected, stop the robot's forward motion
        
        UNDERSTANDING LASER SCAN DATA:
        - self.scan_ranges contains distances to obstacles in meters
        - Each value represents distance at a different angle around the robot
        - Values less than self.stop_distance indicate a close obstacle
        
        CONTROLLING THE ROBOT (Twist message):
        - twist.linear.x: Forward/backward (positive = forward, negative = backward)
        - twist.angular.z: Rotation (positive = left, negative = right)
        - To stop: set twist.linear.x = 0.0 (you can keep angular.z to allow turning)
        
        EXTENSION (optional):
        - For obstacle avoidance, implement turning when obstacles are detected
        - Try to find clear paths by checking different parts of the scan data
        """
        # Filter out invalid readings (very small values, infinity, or NaN)
        valid_ranges = [r for r in self.scan_ranges if not math.isinf(r) and not math.isnan(r) and r > 0.01]
        
        # If no valid readings, assume no obstacles
        if not valid_ranges:
            self.cmd_vel_pub.publish(self.tele_twist)
            return
            
        # Find the closest obstacle in any direction (full 360° scan)
        obstacle_distance = min(valid_ranges)

        # TODO: Implement your obstacle detection logic here!
        # Remember to use obstacle_distance and self.stop_distance in your implementation.
        
        # For now, just use the teleop command (unsafe - replace with your code)
        twist = self.tele_twist

        # Publish the velocity command
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    obstacle_detection = ObstacleDetection()
    rclpy.spin(obstacle_detection)

    obstacle_detection.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
