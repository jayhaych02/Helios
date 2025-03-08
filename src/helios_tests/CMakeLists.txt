cmake_minimum_required(VERSION 3.8)
project(helios_tests)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Copy the robot_kinematics.hpp and .cpp into our include directory
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/../helios_core/include/utils_kinematics/robot_kinematics.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/robot_kinematics.hpp
  COPYONLY
)

# Include directories for test files
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Add kinematics test - include the actual implementation directly
ament_add_gtest(test_kinematics 
  src/test_kinematics.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../helios_core/include/utils_kinematics/robot_kinematics.cpp  
)

target_link_libraries(test_kinematics
  ${rclcpp_LIBRARIES}
)

ament_target_dependencies(test_kinematics
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
)

# Add existing Unity integration test
add_executable(test_unity_node src/TestUnityIntegration.cpp)
target_include_directories(test_unity_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)
ament_target_dependencies(test_unity_node rclcpp std_msgs)

install(TARGETS test_unity_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()