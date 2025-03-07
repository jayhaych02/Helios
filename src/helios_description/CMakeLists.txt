cmake_minimum_required(VERSION 3.8)
project(helios_description)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(ros_gz REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

# Install directories based on your structure
install(DIRECTORY
  meshes
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}")
  ament_python_install_package(${PROJECT_NAME})
endif()

# Install Python executables
install(PROGRAMS
  scripts/teleop_keyboard.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  rclpy
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  urdf
  xacro
  ros_gz
  ros_gz_sim
  ros_gz_bridge
  ros_gz_interfaces
)

# Mark executable files
ament_package()