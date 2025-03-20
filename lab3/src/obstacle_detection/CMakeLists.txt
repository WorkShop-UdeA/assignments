cmake_minimum_required(VERSION 3.5)
project(obstacle_detection)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(turtlebot3_msgs REQUIRED)  # Commented out as it's not needed
# find_package(tf_transformations REQUIRED)  # Commented out as it's not needed
find_package(visualization_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  ${PROJECT_NAME}/obstacle_detection.py
  ${PROJECT_NAME}/lidar_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
