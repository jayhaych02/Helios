# Helios Multi-Robot/Swarm Robotics Disaster Recovery & High-Risk Situation Simulation Learning Tool

## Linux Installation
1. Create robotics directory and navigate to it:
```bash
mkdir robotics && cd robotics
```

2. Clone this repository with its submodules:
```bash
git clone --recurse-submodules https://github.com/YOUR_USERNAME/Helios.git
```
   - If you already cloned without `--recurse-submodules`, run:
     ```bash
     git submodule init
     git submodule update
     ```

3. Navigate to the cloned repo:
```bash
cd Helios
```

4. Make the installation script executable and run it:
```bash
chmod +x install_dependencies.sh
./install_dependencies.sh
```

## ROS Beginners
### Once successfully installed, while inside of the helios directory run:
1. ros2 run demo_nodes_cpp talker
2. In a different terminal: ros2 run demo_nodes_cpp listener
3. In a different terminal: ros2 topic echo /chatter
4. The output is a basic ROS example of Publishing & Subscribing 

## Install Git LFS
### Reasoning : Large codebases will have large files that require lots of space to store. Github by itself isn't able to store them.
### git lfs is an open source Git extension that helps Git repositories manage large binary files efficiently.
### While inside of the helios directory, chmod +x install_git_lfs.sh. Then run ./install_git_lfs.sh inside of the terminal

# GUI Tools Installed : Gazebo & Windows VcXsrv

## Gazebo : "Gazebo Fortress". ROS2 Humble EOL September 2026. Documentation Referenced:
###  https://gazebosim.org/docs/fortress/install_ubuntu/  and https://gazebosim.org/docs/fortress/getstarted/

## Windows VcXsrv : Allows Windows GUI Applications to Function
### https://www.lenovo.com/us/en/glossary/vcxsrv/

# Running Gazebo : After installation scripts have been completed successfully
1. In the Terminal Run : ign gazebo  
2. Select a prebuilt Gazebo Fortress World to mess around with

# Documentation : Doxygen. Coming Soon...

## Next Steps : Creating ROS Packages for major Robot Functionality
1. cd to the /helios/src directory
2. ros2 pkg create --build-type ament_cmake <package_name>
3. Package workflow is : (1) Code a .cpp source file (2) Edit the CMakeLists.txt (3) Edit the package.xml (4) Colcon build

