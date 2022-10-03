
# 📷 Image Path Planning
> ROS2 based local planning package designed to work with RGBD cameras. 

TODO: Image 

The image planner considers optimal trajectories within in the image space, removing the need of local cost map generation. 

## Requirements
- Ubuntu 20.04 
- [ROS2 Foxy Fitzroy](https://docs.ros.org/en/foxy/Installation.html) 
- [Gazebo](https://classic.gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros) 

## Installing 
Within the `src` directory of a ROS2 workspace (e.g. `/colcon_ws`):
```shell
git clone https://github.com/SuryaKannan/image_path_planning.git --recurse-submodules
cd .. && rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Quick Start

TODO: Instructio-set to get a demonstration working 

## Packages

### ROS2 API 

### [waypoint_publisher](/waypoint_generator/waypoint_generator/waypoint_publisher_node.py)

#### Published Topics

`~`

#### Subscribed Topics

`~`

#### Parameters

`~` 

&nbsp;

### [tentacle_planner](/image_planner/image_planner/tentacle_planner_node.py)

#### Published Topics

`~`

#### Subscribed Topics

`~`

#### Parameters

`~`

&nbsp;

### [control](/controller/controller/control_node.py)

#### Published Topics

`~`

#### Subscribed Topics

`~`

#### Parameters

`~`

## Acknowledgements

### [LinoRobot2](https://github.com/linorobot/linorobot2)
- ROS2 port of the [linorobot](https://github.com/linorobot/linorobot) package. Helps to build a custom ROS2 robot (2WD, 4WD, Mecanum Drive) using accessible parts. Able to easily verify experiments on a virtual robot in Gazebo.
### [ros2_numpy](https://github.com/Box-Robotics/ros2_numpy/tree/743513e56e3f35aa91ce07799de4b0f2f59f88c0)
- tools for converting ROS messages to and from numpy arrays for ROS2

## Troubleshooting Guide

| Issue  | Solution |
| ------------- | ------------- |
| Gazebo simulation isn't launching  | Remove the build, log and install files and re-run colcon build WITHOUT the --symlink-install flag. If you are connected to a LAN with other computers running ROS2, make sure you have a different ROS_DOMAIN_ID. The last thing to try is to use ```$ killall gzserver``` in the event there are background processes still running that haven't exited cleanly|
| RuntimeWarning: invalid value encountered in double_scalars m = (point2[1]-point1[1])/(np.cosh(point2[0])-np.cosh(point1[0]))  | Ensure the [number of trajectories chosen](/waypoint_generator/waypoint_generator/waypoint_publisher_node.py) is even. If still this warning appears with no visualisable waypoints in RVIZ, re-run the launch file |

&nbsp;

## Authors

Surya Kannan - thesuryakannan@gmail.com