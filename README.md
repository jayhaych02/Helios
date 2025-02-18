# Helios Multi-Robot/Swarm Robotics Disaster Recovery & High-Risk Situation Simulation Learning Tool

## Linux Installation
1. Create robotics directory and navigate to it:
```bash
mkdir robotics && cd robotics
```

2. Clone this repository with its submodules:
```bash
git clone --recurse-submodules https://github.com/jayhaych02/Helios.git
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

## Gazebo : "Gazebo Ionic". ROS2 Humble EOL September 2026. Documentation Referenced:
### https://gazebosim.org/docs/ionic/install_ubuntu/

## Windows VcXsrv : Allows Windows GUI Applications like Gazebo to function while using WSL
### https://www.lenovo.com/us/en/glossary/vcxsrv/

# Running Gazebo : After installation scripts have been completed successfully
1. In the Terminal Run : gz sim  
2. Select a prebuilt Gazebo Ionic World to mess around with

# Documentation : Doxygen
### In the " ~/robotics/Helios/docs " folder lives doxygen documentation generation "Doxyfile" with folders html and latex

## To view Doxygen Documentation
1. sudo apt install doxygen
2. cd ~/robotics/Helios/docs
3. Run " python3 -m http.server 8000 "
4. Paste the port " http://localhost:8000/" into the browser or open the port while inside of WSL VsCode

 

# Helios Multi-Robot/Swarm Robotics Disaster Recovery & High-Risk Situation Simulation Learning Tool

![Gazebo Demonstration](./GazeboRunning.png)

## Project Demonstrations
- [Watch Unity Demo Video](./UNITY.mp4)
- [Watch Foxglove Demo Video](./Foxglove.mp4)

