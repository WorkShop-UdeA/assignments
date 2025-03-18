#!/usr/bin/env python3
import rclpy
import ast
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class TurtleBot(Node):
    def __init__(self):
        super().__init__("turtlesim_waypoints")

        # Set up parameters
        self.setup_parameters()

        # Publisher for velocity commands
        self.velocity_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        # Subscriber for turtle position
        self.pose_subscriber = self.create_subscription(
            Pose, "/turtle1/pose", self.update_pose, 10
        )

        # Initialize attributes
        self.pose = Pose()
        self.timer = self.create_timer(0.1, self.controller_callback)
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.goal_pose = Pose()
        self.moving_to_goal = False
        self.distance_tolerance = 0.0
        self.path_completed = False

        # Parse waypoints and start if auto_start is true
        self.parse_waypoints()
        if self.get_parameter("auto_start").value:
            self.get_logger().info("Auto-starting waypoint navigation")
            self.start_movement()

    def setup_parameters(self):
        """Setup node parameters"""
        self.declare_parameter(
            "waypoints",
            "[[1.0, 1.0], [9.0, 1.0], [9.0, 9.0], [1.0, 9.0], [5.5, 5.5]]",
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="List of waypoints in format [[x1,y1], [x2,y2], ...]",
            ),
        )
        self.declare_parameter(
            "distance_tolerance",
            0.5,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="How close the turtle needs to get to a waypoint",
            ),
        )
        self.declare_parameter(
            "auto_start",
            True,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="Whether to start navigation automatically",
            ),
        )
        self.declare_parameter(
            "loop_waypoints",
            False,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="Whether to continuously loop through waypoints",
            ),
        )

        # Movement parameters for smoother transitions
        self.declare_parameter(
            "max_linear_speed",
            2.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE, description="Maximum linear speed"
            ),
        )
        self.declare_parameter(
            "min_linear_speed",
            0.5,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE, description="Minimum linear speed"
            ),
        )
        self.declare_parameter(
            "linear_acceleration",
            1.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Linear acceleration factor",
            ),
        )
        self.declare_parameter(
            "angular_speed_factor",
            3.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Angular speed adjustment factor",
            ),
        )
        self.declare_parameter(
            "lookahead_distance",
            1.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Distance to look ahead on path for smoother turns",
            ),
        )

        # Add parameter callback
        self.add_on_set_parameters_callback(self.parameters_callback)

    def update_pose(self, data):
        """Callback for pose updates"""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def euclidean_distance(self, goal_pose):
        """Calculate distance to goal"""
        return sqrt(
            pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2)
        )

    def linear_vel(self, goal_pose):
        """Calculate linear velocity with acceleration and deceleration"""
        # Get parameters
        max_speed = self.get_parameter("max_linear_speed").value
        min_speed = self.get_parameter("min_linear_speed").value

        # Calculate distance to goal
        distance = self.euclidean_distance(goal_pose)

        # Deceleration zone is 2x the distance_tolerance
        decel_zone = self.get_parameter("distance_tolerance").value * 2.0

        if distance < decel_zone:
            # Decelerate as we approach the goal
            # This ensures smoother stops at waypoints
            speed = min_speed + (max_speed - min_speed) * (distance / decel_zone)
        else:
            # Normal speed
            speed = max_speed

        # Ensure speed stays within bounds
        return max(min_speed, min(speed, max_speed))

    def steering_angle(self, goal_pose):
        """Calculate steering angle"""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        """Calculate angular velocity with smoother turning"""
        # Get angular speed factor
        angular_factor = self.get_parameter("angular_speed_factor").value

        # Calculate angle difference
        angle_diff = self.steering_angle(goal_pose) - self.pose.theta

        # Normalize angle between -pi and pi
        while angle_diff > 3.14159:
            angle_diff -= 2 * 3.14159
        while angle_diff < -3.14159:
            angle_diff += 2 * 3.14159

        # Apply non-linear scaling for smoother turns
        # Use a different approach to avoid circular patterns
        # For smaller angles, apply proportional control
        # For larger angles, apply stronger corrections but still smooth
        if abs(angle_diff) < 0.1:  # Very small angle difference - gentle correction
            return angle_diff * angular_factor * 0.8
        elif abs(angle_diff) < 0.5:  # Medium angle difference - normal correction
            return angle_diff * angular_factor
        else:  # Large angle difference - stronger correction to avoid loops
            return angle_diff * angular_factor * 1.2

    def parameters_callback(self, params):
        """Handle parameter changes"""
        for param in params:
            self.get_logger().info(f"Parameter {param.name} changed to {param.value}")

            # Handle parameter changes
            if param.name == "waypoints" and not self.moving_to_goal:
                self.parse_waypoints()

            elif param.name == "auto_start" and param.value:
                if not self.moving_to_goal:
                    self.parse_waypoints()
                    self.start_movement()

        return True

    def parse_waypoints(self):
        """Parse waypoints from string parameter using ast.literal_eval"""
        self.distance_tolerance = self.get_parameter("distance_tolerance").value
        waypoints_str = self.get_parameter("waypoints").value

        try:
            waypoints_data = ast.literal_eval(waypoints_str)
            if not isinstance(waypoints_data, list):
                raise ValueError("Waypoints must be a list")

            # Clear existing waypoints
            self.waypoints = []

            # Process waypoints
            for i, coords in enumerate(waypoints_data):
                if isinstance(coords, list) and len(coords) == 2:
                    wp = Pose()
                    wp.x = float(coords[0])
                    wp.y = float(coords[1])
                    self.waypoints.append(wp)
                    self.get_logger().info(f"Loaded waypoint {i + 1}: ({wp.x}, {wp.y})")
                else:
                    self.get_logger().warn(f"Invalid waypoint format at index {i}")

            if self.waypoints:
                self.get_logger().info(
                    f"Successfully loaded {len(self.waypoints)} waypoints"
                )
                self.current_waypoint_idx = 0
                self.path_completed = False

                # Set initial goal
                self.goal_pose = self.waypoints[0]
            else:
                self.get_logger().error(
                    "No valid waypoints found. Please check waypoints parameter."
                )

        except (SyntaxError, ValueError) as e:
            self.get_logger().error(f"Error parsing waypoints: {e}")

    def start_movement(self):
        """Start movement to current waypoint"""
        if not self.waypoints:
            self.get_logger().error("No waypoints available. Cannot start movement.")
            return

        wp = self.waypoints[self.current_waypoint_idx]
        self.goal_pose = wp
        self.get_logger().info(
            f"Moving to waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)}: "
            f"x={wp.x}, y={wp.y}"
        )

        self.moving_to_goal = True

    def advance_to_next_waypoint(self):
        """Move to the next waypoint"""
        if not self.waypoints:
            self.get_logger().warn("No waypoints available.")
            self.moving_to_goal = False
            return
        self.current_waypoint_idx += 1

        # Check if we've completed all waypoints
        if self.current_waypoint_idx >= len(self.waypoints):
            if self.get_parameter("loop_waypoints").value:
                self.get_logger().info("Path completed, looping back to start")
                self.current_waypoint_idx = 0
            else:
                self.get_logger().info("Path completed!")
                self.moving_to_goal = False
                self.path_completed = True
                return

        # Set new goal and continue
        wp = self.waypoints[self.current_waypoint_idx]
        self.goal_pose = wp
        self.get_logger().info(
            f"Moving to waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)}: "
            f"x={wp.x}, y={wp.y}"
        )

    def controller_callback(self):
        """Main control loop with improved path following"""
        # Skip if not moving or no waypoints
        if not self.moving_to_goal or not self.waypoints:
            return

        vel_msg = Twist()

        # Check if we've reached the current goal
        if self.euclidean_distance(self.goal_pose) >= self.distance_tolerance:
            # Look ahead along the path if there's a next waypoint
            target_pose = self.goal_pose

            # Check if we should use lookahead for smoother cornering
            if (
                self.current_waypoint_idx < len(self.waypoints) - 1
                and self.get_parameter("lookahead_distance").value > 0.0
            ):
                # Distance to current waypoint
                dist_to_current = self.euclidean_distance(self.goal_pose)
                lookahead_dist = self.get_parameter("lookahead_distance").value

                # Adjust lookahead based on angle to reduce circular paths
                # Get the current waypoint and next waypoint
                current_wp = self.goal_pose
                next_wp = self.waypoints[self.current_waypoint_idx + 1]

                # Vector from robot to current waypoint
                v1_x = current_wp.x - self.pose.x
                v1_y = current_wp.y - self.pose.y

                # Vector from current waypoint to next waypoint
                v2_x = next_wp.x - current_wp.x
                v2_y = next_wp.y - current_wp.y

                # Normalize vectors (if not zero)
                v1_len = sqrt(v1_x * v1_x + v1_y * v1_y)
                v2_len = sqrt(v2_x * v2_x + v2_y * v2_y)

                # Only consider lookahead if we're close enough and path direction is changing
                if dist_to_current < lookahead_dist and v1_len > 0 and v2_len > 0:
                    # Normalize vectors
                    v1_x /= v1_len
                    v1_y /= v1_len
                    v2_x /= v2_len
                    v2_y /= v2_len

                    # Dot product gives cosine of angle between vectors
                    dot_product = v1_x * v2_x + v1_y * v2_y

                    # If dot product is near 1, paths are similar direction
                    # If dot product is near -1, paths are opposite direction
                    # If dot product is near 0, paths are perpendicular

                    # Only blend if we're changing direction significantly
                    if dot_product < 0.7:  # Angle > ~45 degrees
                        # Calculate a blend factor based on distance and angle
                        blend_factor = max(
                            0.0,
                            min(
                                0.7,
                                (1.0 - dot_product)
                                * 0.5
                                * (1.0 - dist_to_current / lookahead_dist),
                            ),
                        )

                        # Create a temporary target that's a blend between current and next waypoint
                        blend_pose = Pose()
                        blend_pose.x = (
                            current_wp.x * (1.0 - blend_factor)
                            + next_wp.x * blend_factor
                        )
                        blend_pose.y = (
                            current_wp.y * (1.0 - blend_factor)
                            + next_wp.y * blend_factor
                        )

                        # Use the blended target for control
                        target_pose = blend_pose

            # Adjust linear velocity based on angular deviation to slow down on turns
            angular_diff = abs(self.steering_angle(target_pose) - self.pose.theta)
            while angular_diff > 3.14159:
                angular_diff = abs(angular_diff - 2 * 3.14159)

            # If we're turning sharply, reduce linear velocity
            turn_factor = 1.0
            if angular_diff > 0.5:  # ~30 degrees
                turn_factor = max(0.4, 1.0 - angular_diff / 3.14159)

            # Calculate control signals with the potentially modified target
            base_linear_vel = self.linear_vel(target_pose)
            vel_msg.linear.x = base_linear_vel * turn_factor
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = self.angular_vel(target_pose)

            # Send control signals
            self.velocity_publisher.publish(vel_msg)
        else:
            # Stop when goal reached
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.velocity_publisher.publish(vel_msg)

            # Move to next waypoint
            self.advance_to_next_waypoint()


def main(args=None):
    rclpy.init(args=args)

    turtlesim = TurtleBot()

    try:
        rclpy.spin(turtlesim)
    except KeyboardInterrupt:
        turtlesim.get_logger().info("Keyboard interrupt detected, shutting down")
    finally:
        turtlesim.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

