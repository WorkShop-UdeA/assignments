from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory and config file path
    package_dir = get_package_share_directory('turtlesim_waypoints')
    config_file = os.path.join(package_dir, 'config', 'turtlesim_goals.yaml')
    
    # Define nodes
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        output='screen'
    )
    
    turtlesim_waypoints_node = Node(
        package='turtlesim_waypoints',
        executable='gotogoal.py',
        name='turtlesim_waypoints',
        parameters=[config_file],
        output='screen'
    )
    
    # Delay controller start to ensure turtlesim is ready
    delayed_controller = TimerAction(
        period=1.0,
        actions=[turtlesim_waypoints_node]
    )
    
    # Return the launch description
    return LaunchDescription([
        turtlesim_node,
        delayed_controller
    ])