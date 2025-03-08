cmake_minimum_required(VERSION 3.8)
project(helios_description)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Install directories based on your structure
install(DIRECTORY
  meshes
  rviz
  urdf
  scripts
  DESTINATION share/${PROJECT_NAME}
)



# Install  executables
install(PROGRAMS
  scripts/teleop_keyboard.cpp
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  urdf
  xacro
  gazebo_ros
)

# Mark executable files
ament_package()