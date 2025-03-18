cmake_minimum_required(VERSION 3.5)
project(turtlesim_goal)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  ${PROJECT_NAME}/gotogoal.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
