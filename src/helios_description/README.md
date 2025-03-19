# Helios Robot Description Package

This package contains URDF/Xacro files and launch configurations for the Helios robot models, including Firefighter, Scout, Heavy-Duty, and Medical Support robots. These models use a differential drive system and are designed to work with Gazebo and ROS Humble.

## Files Included
- **Model Files**:
  - `urdf/robots/firefighter_robot.urdf.xacro`
  - `urdf/robots/scout_robot.urdf.xacro`
  - `urdf/robots/heavy_duty_robot.urdf.xacro`
  - `urdf/robots/medical_support_robot.urdf.xacro`
- **Base Differential Drive**:
  - `urdf/base_differential_drive.xacro` (base template for differential drive)
- **Gazebo Control**:
  - `urdf/gazebo_control.xacro` (alternative Gazebo plugin configuration, not currently used in models)
- **Launch Files**:
  - `helios_description/launch/description.launch.py` (validation with `robot_state_publisher`)
  - `helios_bringup/launch/spawn_<robot_name>.launch.py` (Gazebo spawning, one per robot: `firefighter`, `scout`, `heavy_duty`, `medical_support`)
- **Configuration**:
  - `CMakeLists.txt` (build configuration)
  - `package.xml` (package dependencies)

## Prerequisites
- **ROS Humble**: Ensure ROS Humble is installed.
- **Gazebo**: Compatible with Gazebo Sim 7 (tested) or later versions (e.g., Sim 8 or Harmonic). Note: Sim 7 has a known plugin issue (DART defaulting instead of Bullet); testing with other versions is ongoing.
- **Dependencies**: Install `launch`, `launch_ros`, `robot_state_publisher`, `xacro`, and `ros_gz_sim` packages.

## Usage
1. **Clone the Repository**:
   ```bash
   cd ~/robotics
   git clone <repository_url> Helios
   cd Helios


Build the Workspace:

colcon build --allow-overriding helios_description helios_bringup
source install/setup.bash


Validate a Model: Convert the Xacro to URDF and check:

ros2 run xacro xacro src/helios_description/urdf/robots/<robot_name>.urdf.xacro > /tmp/<robot_name>.urdf
check_urdf /tmp/<robot_name>.urdf


Launch with robot_state_publisher to verify:


ros2 launch helios_description description.launch.py model:=<robot_name>


Spawn in Gazebo (Experimental): Use the individual launch files (requires a working Gazebo setup):


ros2 launch helios_bringup spawn_<robot_name>.launch.py