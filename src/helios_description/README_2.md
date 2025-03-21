Prerequisites
Unity: Version 6.0.0.41f1 with ROS-TCP-Connector package.
ROS2: Installed on WSL2 with a workspace (~/robotics/Helios).
Xacro Files: Located in ~/robotics/Helios/src/helios_description/urdf/

Navigate to the URDF directory in WSL:
cd ~/robotics/Helios/src/helios_description/urdf

Convert each Xacro file to URDF:
xacro firefighter_robot.xacro > firefighter_robot.urdf
xacro heavy_duty_robot.xacro > heavy_duty_robot.urdf
xacro medical_support_robot.xacro > medical_support_robot.urdf
xacro scout_robot.xacro > scout_robot.urdf

Alternative: the new unity xacro robot files

Step 2: Copy URDF Files to Unity Project
In Unity Editor, create a folder Assets/Robots.
Copy the URDF files (firefighter_robot.urdf, heavy_duty_robot.urdf, medical_support_robot.urdf, scout_robot.urdf) from WSL to Assets/Robots/.

Step 3: Import URDF Files into Unity
In Unity, in Assets/Robots, right-click each URDF file and select "Import Robot from Selected URDF file".
Set Axis Type: Z Axis.
Set Convex Decomposer: VHACD.
Click "Import URDF".
Drag each robot prefab into the OutdoorsScene in the Hierarchy:
firefighter_robot: X=0, Y=0, Z=15
heavy_duty_robot: X=2, Y=0, Z=15
medical_support_robot: X=4, Y=0, Z=15
scout_robot: X=6, Y=0, Z=15
Adjust Y-position (e.g., Y=0.217) to place on terrain.


Step 4: Add Differential Drive Controller Script
In Unity, add the script named DifferentialDriveController.cs in Assets (script available on GitHub).
Attach the script to each robot in the OutdoorsScene:
Add the DifferentialDriveController component.
Assign Left Wheel and Right Wheel fields to left_wheel and right_wheel GameObjects under each robot.
Set Wheel Separation to 0.4 and Wheel Radius to 0.05.

Step 5: Set Up ROS-TCP-Endpoint Server
In a WSL terminal, run the ROS-TCP-Endpoint server:
cd ~/robotics/Helios
source install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=Your IP Address  

In Unity, go to Robotics > ROS Settings:
Set Host to Your IP Address.
Set Port to 10000.


Step 6: Test Movement with a /cmd_vel Message
In Unity, enter Play Mode.
In a second WSL terminal, publish a /cmd_vel message:
cd ~/robotics/Helios
source install/setup.bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --times 50
Verify the robots move forward 5 meters in Unity.


Step 7: Set Up Continuous Movement with a ROS Node
In WSL, create a new ROS package
cd ~/robotics/Helios/src
ros2 pkg create --build-type ament_python robot_controller --dependencies rclpy geometry_msgs

In robot_controller/robot_controller, create cmd_vel_publisher.py (script available on GitHub).
Update setup.py in robot_controller to add the entry point:
entry_points={
    'console_scripts': [
        'cmd_vel_publisher = robot_controller.cmd_vel_publisher:main',
    ],
},

Build the workspace:
cd ~/robotics/Helios
colcon build
source install/setup.bash

Run the node:
ros2 run robot_controller cmd_vel_publisher



Step 8: Add Turning Motion
Modify cmd_vel_publisher.py to add turning:
Set self.angular_speed = 0.5 in the __init__ method.


Rebuild the workspace:
cd ~/robotics/Helios
colcon build
source install/setup.bash


Run the modified node:
ros2 run robot_controller cmd_vel_publisher


In Unity, reposition the Main Camera to observe firefighter_robot
Enter Play Mode to observe the robot’s movement.
