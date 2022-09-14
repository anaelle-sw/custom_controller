# Custom_pure_pursuit_controller
This package contains the sources of the custom controller and of the custom progress checker (both exported as Navigation2 controller plugins).
It also feature a basic test on the controller.

## The controller
The controller is based on [Navigation2 pure pursuit controller](https://github.com/ros-planning/navigation2_tutorials/tree/126902457c5c646b136569886d6325f070c1073d/nav2_pure_pursuit_controller), and is using collision detection from [Navigation2 regulated pure puresuit controller](https://github.com/ros-planning/navigation2/tree/galactic/nav2_regulated_pure_pursuit_controller).
It is compatible with Navigation2 API.
It has been tested with turtlebot3 burger simulation.

## The progress checker
The progress checker is based on [Navigation2 simple progress checker](https://github.com/ros-planning/navigation2/tree/main/nav2_controller/plugins).
It also publishes on change a `ProgressStatus` message on topic `/follow_path/progress_status` according to the robot state while navigating:
* Following a new path
* Progressing
* Stuck
