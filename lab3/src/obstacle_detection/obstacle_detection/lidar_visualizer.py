#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import math
import numpy as np
from rclpy.time import Time
from rclpy.duration import Duration


class LidarVisualizer(Node):
    """
    Educational node to visualize LiDAR readings for better understanding of sensor data.
    Creates visual markers showing the full 360-degree LiDAR readings and a circular boundary.
    """
    def __init__(self):
        super().__init__("lidar_visualizer")
        
        # Parameters
        self.declare_parameter("stop_distance", 0.35)
        self.stop_distance = self.get_parameter("stop_distance").get_parameter_value().double_value
        
        # Visualization properties
        self.declare_parameter("range_multiplier", 1.5)
        multiplier = self.get_parameter("range_multiplier").get_parameter_value().double_value
        
        # Show max distance as a multiple of stop_distance
        self.max_distance = self.stop_distance * multiplier  # Show 1.5x the stop distance by default
        
        self.scan_ranges = []
        self.scan_header = None
        self.has_scan_received = False
        
        # Performance settings to fix blinking
        self.declare_parameter("marker_lifetime", 0.5)  # seconds
        self.marker_lifetime = self.get_parameter("marker_lifetime").get_parameter_value().double_value
        
        # Use a reliable QoS profile for the markers
        qos = QoSProfile(depth=10)
        
        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(MarkerArray, "lidar_markers", qos)

        # Subscribe to laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan, "scan", self.scan_callback, qos_profile=qos_profile_sensor_data
        )

        # Timer for regular visualization updates - reduce the rate to avoid overwhelming RViz
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info(f"LiDAR Visualizer started. Stop distance: {self.stop_distance}m, " 
                               f"Max visualization range: {self.max_distance}m")

    def scan_callback(self, msg):
        """Process incoming LaserScan messages"""
        self.scan_ranges = msg.ranges
        self.scan_header = msg.header
        self.scan_angle_min = msg.angle_min
        self.scan_angle_max = msg.angle_max
        self.scan_angle_increment = msg.angle_increment
        self.total_angles = len(self.scan_ranges)
        self.has_scan_received = True

    def timer_callback(self):
        """Periodic function to update and publish visualization markers"""
        if self.has_scan_received:
            self.publish_markers()
    
    def publish_markers(self):
        """Create and publish visualization markers for LiDAR data"""
        marker_array = MarkerArray()
        
        # Find minimum distance in the entire 360 scan, ignoring invalid readings
        valid_ranges = [r for r in self.scan_ranges if not math.isinf(r) and r > 0.01]
        min_distance = min(valid_ranges, default=float('inf'))
        
        # Check if obstacle is detected
        obstacle_detected = min_distance < self.stop_distance
        
        # Create a circular visualization for the LiDAR area
        circle_marker = self.create_circle_marker(obstacle_detected)
        marker_array.markers.append(circle_marker)
        
        # Add a boundary marker to show the stop distance
        boundary_marker = self.create_boundary_marker()
        marker_array.markers.append(boundary_marker)
        
        # Add a text marker to show distance information
        text_marker = self.create_text_marker(min_distance, obstacle_detected)
        marker_array.markers.append(text_marker)
        
        # Publish all markers
        self.marker_pub.publish(marker_array)
    
    def create_circle_marker(self, obstacle_detected):
        """Create a circular marker showing LiDAR readings all around the robot"""
        circle_marker = Marker()
        
        # Important for TF stability - use the same header as the scan
        if self.scan_header:
            circle_marker.header = self.scan_header
        else:
            circle_marker.header.frame_id = "base_link"
            circle_marker.header.stamp = self.get_clock().now().to_msg()
            
        circle_marker.ns = "lidar_visualization"
        circle_marker.id = 0
        circle_marker.type = Marker.TRIANGLE_LIST
        circle_marker.action = Marker.ADD
        
        # Set lifetime to prevent blinking/lingering markers
        duration = Duration(seconds=self.marker_lifetime)
        circle_marker.lifetime = duration.to_msg()
        
        # Basic properties
        circle_marker.scale.x = 1.0
        circle_marker.scale.y = 1.0
        circle_marker.scale.z = 1.0
        
        # Color based on obstacle detection (red if obstacle detected, green otherwise)
        if obstacle_detected:
            circle_marker.color.r = 1.0
            circle_marker.color.g = 0.0
            circle_marker.color.b = 0.0
            circle_marker.color.a = 0.5  # Semi-transparent
        else:
            circle_marker.color.r = 0.0
            circle_marker.color.g = 1.0
            circle_marker.color.b = 0.0
            circle_marker.color.a = 0.3  # More transparent when no obstacle
        
        # Create triangles for the full 360-degree circle
        # Add origin point at the center of the robot
        origin = Point(x=0.0, y=0.0, z=0.05)
        
        # Skip some indices to reduce visual noise
        step = max(1, self.total_angles // 60)  # Use at most 60 points for the full 360-degree circle
        
        # Generate triangles for each point in the scan
        for i in range(0, self.total_angles - step, step):
            try:
                # Get current point
                idx_current = i
                angle_current = self.scan_angle_min + idx_current * self.scan_angle_increment
                distance_current = min(self.scan_ranges[idx_current], self.max_distance)
                
                # Skip invalid readings (0.0 or infinity)
                if distance_current <= 0.01 or math.isinf(distance_current):
                    continue
                
                p1 = Point()
                p1.x = distance_current * math.cos(angle_current)
                p1.y = distance_current * math.sin(angle_current)
                p1.z = 0.05  # Slightly above ground
                
                # Get next point
                idx_next = (i + step) % self.total_angles
                angle_next = self.scan_angle_min + idx_next * self.scan_angle_increment
                distance_next = min(self.scan_ranges[idx_next], self.max_distance)
                
                # Skip invalid readings
                if distance_next <= 0.01 or math.isinf(distance_next):
                    continue
                
                p2 = Point()
                p2.x = distance_next * math.cos(angle_next)
                p2.y = distance_next * math.sin(angle_next)
                p2.z = 0.05
                
                # Add triangle points (origin, p1, p2)
                circle_marker.points.append(origin)
                circle_marker.points.append(p1)
                circle_marker.points.append(p2)
            except IndexError:
                # Skip any index errors from misaligned data
                continue
        
        return circle_marker
    
    def create_boundary_marker(self):
        """Create a circular boundary marker showing the stop distance threshold"""
        boundary_marker = Marker()
        
        # Important for TF stability - use the same header as the scan
        if self.scan_header:
            boundary_marker.header = self.scan_header
        else:
            boundary_marker.header.frame_id = "base_link"
            boundary_marker.header.stamp = self.get_clock().now().to_msg()
            
        boundary_marker.ns = "lidar_visualization"
        boundary_marker.id = 1
        boundary_marker.type = Marker.LINE_STRIP
        boundary_marker.action = Marker.ADD
        
        # Set lifetime to prevent blinking/lingering markers
        duration = Duration(seconds=self.marker_lifetime)
        boundary_marker.lifetime = duration.to_msg()
        
        boundary_marker.scale.x = 0.03  # Line width
        boundary_marker.color.r = 1.0
        boundary_marker.color.g = 1.0
        boundary_marker.color.b = 0.0  # Yellow
        boundary_marker.color.a = 1.0
        
        # Create a full circle at the stop_distance
        num_points = 60
        # 360 degrees total
        angle_range = 2 * math.pi
        
        for i in range(num_points + 1):
            angle = i * angle_range / num_points
            point = Point()
            point.x = self.stop_distance * math.cos(angle)
            point.y = self.stop_distance * math.sin(angle)
            point.z = 0.05
            boundary_marker.points.append(point)
        
        return boundary_marker
    
    def create_text_marker(self, distance, obstacle_detected):
        """Create a text marker showing the minimum distance information"""
        text_marker = Marker()
        
        # Important for TF stability - use the same header as the scan
        if self.scan_header:
            text_marker.header = self.scan_header
        else:
            text_marker.header.frame_id = "base_link"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            
        text_marker.ns = "lidar_visualization"
        text_marker.id = 2
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        
        # Set lifetime to prevent blinking/lingering markers
        duration = Duration(seconds=self.marker_lifetime)
        text_marker.lifetime = duration.to_msg()
        
        # Position the text above the robot
        text_marker.pose.position.x = 0.0
        text_marker.pose.position.y = 0.0
        text_marker.pose.position.z = 0.3
        
        text_marker.scale.z = 0.1  # Text height
        
        # Color based on obstacle detection
        if obstacle_detected:
            text_marker.color.r = 1.0
            text_marker.color.g = 0.0
            text_marker.color.b = 0.0
            text_marker.text = f"OBSTACLE: {distance:.2f}m"
        else:
            text_marker.color.r = 0.0
            text_marker.color.g = 1.0
            text_marker.color.b = 0.0
            text_marker.text = f"Distance: {distance:.2f}m"
        
        text_marker.color.a = 1.0
        
        return text_marker


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    lidar_visualizer = LidarVisualizer()
    rclpy.spin(lidar_visualizer)

    lidar_visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main() 