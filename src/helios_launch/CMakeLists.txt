cmake_minimum_required(VERSION 3.8)
project(helios_launch)

find_package(ament_cmake REQUIRED)

include_directories(include)

install(
  DIRECTORY launch config worlds 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()