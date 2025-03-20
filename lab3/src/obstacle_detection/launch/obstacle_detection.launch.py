from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the launch argument with a default value
    stop_distance_arg = DeclareLaunchArgument(
        'stop_distance',
        default_value='0.25',
        description='Distance in meters at which the robot stops when obstacles are detected'
    )
    
    # Use LaunchConfiguration to reference the argument
    stop_distance = LaunchConfiguration('stop_distance')
    
    obstacle_detection_cmd = Node(
        package='obstacle_detection',
        executable='obstacle_detection.py',
        name='obstacle_detection',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'stop_distance': stop_distance}
        ],
    )
    
    # Launch the lidar visualizer node 
    lidar_visualizer_cmd = Node(
        package='obstacle_detection',
        executable='lidar_visualizer.py',
        name='lidar_visualizer',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'stop_distance': stop_distance} 
        ],
    )
    
    # Create and return launch description
    ld = LaunchDescription()
    ld.add_action(stop_distance_arg)  # Add the declaration first
    ld.add_action(obstacle_detection_cmd)
    ld.add_action(lidar_visualizer_cmd)
    
    return ld 