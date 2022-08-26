
# 📷 Image Path Planning
> ROS2 based local planning package designed to work with RGBD cameras. 

TODO: Image 

The image planner considers optimal trajectories within in the image space, removing the need of local cost map generation. 

## Requirements
- Ubuntu 20.04 
- [ROS2 Foxy Fitzroy](https://docs.ros.org/en/foxy/Installation.html)
- [Gazebo](https://classic.gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros) (optional)

## Installing 
Within the `src` directory of a ROS2 workspace (e.g. `/colcon_ws`):
```shell
git clone https://github.com/SuryaKannan/image_path_planning.git
cd ~/colcon_ws
rosdep install -i --from-path src --rosdistro foxy -y
colcon build
```
Do not forget to source the workspace.

## Quick Start

TODO: Instruction set to get a demonstration working 

## Packages

TODO: Summary of package functionalities (both developed and forked) 

## ROS2 API 

### 1. Node Name 

#### Published Topics

`~`

#### Subscribed Topics

`~`

#### Parameters

`~`

## Troubleshooting Guide

TODO: For all commonly issues found 

## Authors

Surya Kannan - surya.kannan@icloud.com