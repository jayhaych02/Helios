# Helios Multi-Robot/Swarm Robotics Disaster Recovery & High-Risk Situation Simulation Learning Tool

## Linux Installation
1. mkdir robotics & cd robotics
2. git clone this repo in your base Linux Environment path
3. cd to the cloned repo
4. chmod +x install_dependencies.sh
5. In the terminal : ./install_dependencies.sh


## ROS Beginners
### Once successfully installed, while inside of the helios directory run:
1. ros2 run demo_nodes_cpp talker
2. In a different terminal: ros2 run demo_nodes_cpp listener
3. In a different terminal: ros2 topic echo /chatter
4. The output is a basic ROS example of Publishing & Subscribing 


## Install Git LFS
### Reasoning : Large codebases will have large files that require lots of space to store. Github by itself isn't able to store them.
### git lfs is an open source Git extension that helps Git repositories manage large binary files efficiently.
### While inside of the helios directory, chmod +x install_git_lfs.sh. Then ./install_git_lfs.sh inside of the terminal


## Documentation : Doxygen. Coming soon...

## Next Steps : Creating ROS Packages for major Robot Functionality
1. cd to the /helios/src directory
2. ros2 pkg create --build-type ament_cmake <package_name> --dependencies <dependencies>
3. Package workflow is : (1) Code a .cpp source file (2) Edit the CMakeLists.txt (3) Edit the package.xml (4) Colcon build 
