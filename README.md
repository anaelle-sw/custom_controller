# Custom controller
A custom controller for nav2 and turtlebot3, based on nav2's pure pursuit controller

## Prequisites
* Install [ROS2 Galatic](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)
* Install [Nav2](https://navigation.ros.org/build_instructions/index.html): `sudo apt install ros-galactic-navigation2 ros-galactic-nav2-bringup`
* Install [TurtleBot3](https://github.com/ROBOTIS-GIT/turtlebot3/tree/galactic-devel): `sudo apt install ros-galactic-turtlebot3*`
* Install Colcon: `sudo apt install python3-colcon-common-extensions`

## Install project
* Git clone this project: `git clone https://github.com/anaelle-sw/custom_controller.git custom_controller_ws/src`
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
