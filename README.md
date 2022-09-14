# Custom controller
A custom controller plugin for Navigation2 API (ROS2 Galactic).
The controller has been tested with turtlebot3 burger simulation.
It is based on [Navigation2 pure pursuit controller](https://github.com/ros-planning/navigation2_tutorials/tree/126902457c5c646b136569886d6325f070c1073d/nav2_pure_pursuit_controller), and is using collision detection from [Navigation2 regulated pure puresuit controller](https://github.com/ros-planning/navigation2/tree/galactic/nav2_regulated_pure_pursuit_controller).

## Packages
This repository contains three packages.
* `turtlebot_bringup`: all configuration files and launch files to bringup the custom controller with a turtlebot3 burger simulation
* `custom_pure_pursuit_controller`: the sources of the custom controller and of the custom progress checker (both exported as Navigation2 controller plugins)
* `custom_msgs`: custom message definition for the progress checker to publish

## Prequisites
* Install [ROS2 Galatic](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)
* Install [Nav2](https://navigation.ros.org/build_instructions/index.html): `sudo apt install ros-galactic-navigation2 ros-galactic-nav2-bringup`
* Install [TurtleBot3](https://github.com/ROBOTIS-GIT/turtlebot3/tree/galactic-devel): `sudo apt install ros-galactic-turtlebot3*`
* Install Colcon: `sudo apt install python3-colcon-common-extensions`

## Install project
* Clone this project: `git clone https://github.com/anaelle-sw/custom_controller.git custom_controller_ws/src`
* Build this project:
```
cd custom_controller_ws
colcon build
```

## Launch project
* Source ROS2 and project:
```
source /opt/ros/galactic/setup.bash
source ~/custom_controller_ws/install/local_setup.bash
```
* Export the path to the Gazebo models for Turtlebots: `export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/galactic/share/turtlebot3_gazebo/models`
* Launch project: `ros2 launch turtlebot_bringup simulation_bringup.launch.py`

## Use the custom controller
* Using Rviz's plugin "2D Pose Estimate", localize the robot approximately here:  
* The robot should now be localized and the local and global costmap should be displayed:
* Usign Rviz's plugin "Nav2 Goal", send the robot anywhere in the map:
* The robot should follow the diplayed global path, usign the custom controller:
* You can test the collision detection by generating a box via Gazebo. The box must be paced on the global path, while the robot is moving: 
