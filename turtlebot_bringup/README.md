# Turtlebot_bringup
This package contains all configuration files and launch files to bringup the custom controller with a turtlebot3 burger simulation.

## Behavior trees
* `simple_navigate_to_pose.xml`: a behavior tree using the custom controller to follow a global path (with no recovery nor retries)

## Launch
* `localization.launch.py`: a launch file for Navigation2 localization nodes
* `navigation.launch.py`: a launch file for Navigation2 navigation nodes, with custom parameters transmitted to the controller server
* `rviz.launch.py`: a launch file for Rviz, using a standard Rviz configuration
* `simulation_bringup.launch.py`: a launch file for loading a Gazebo simulation of the turtlebot3 burger model in a world. Also launches `localization.launch.py`, `navigation.launch.py`, and `rviz.launch.py`

## Maps
* `turtlebot3_world`: the standard world for turtlebot3 simulation, for Navigation2 map

## Params
* `default_nav2_params.yaml`: the default parameters for Navigation2 stack, transmitted to all Navigation2 nodes launched except the controller server.
* `custom_controller_params.yaml`: the custom parameters transmitted to the controller server.

## Rviz
* `nav2_default_view.rviz`: a standard rviz configuration

## URDF
* `turtlebot3_burger.urdf`: the URDF for the turtlebot3 burger model, transmitted to the robot state publisher

## Worlds
* `turtlebot3_world_and_burger.model`: the standard world for turtlebot3 simulation and the model for turtlebot3 burger, for Gazebo
