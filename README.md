[![Image](https://img.shields.io/badge/ROS-Noetic-purple.svg)](https://github.com/arthurgomes4)
[![Image](https://img.shields.io/badge/Gazebo-11.9.0-orange.svg)](https://github.com/arthurgomes4)

# Omni Wheels Simulation

This repository contains ROS packages for simulating omni-wheel mobile bases in Gazebo with realistic wheel-ground interaction.

## Introduction
Omni wheels or poly wheels are wheels with freely rotating rollers around the circumference which are perpendicular to the turning direction. The effect is that the wheel can be driven with full force, but will also slide laterally with great ease. These wheels are often employed in holonomic drive systems.

<p align="center">
  <img src="./README_images/omni.jpeg" width="200" title="bot">
</p>
<p align="center">
    Fig: 100mm omni wheel
</p>

## List of packages
- [omni_wheel_description](./omni_wheel_description): Contains description Xacro files for various omni-wheels. 
- [omni_wheel_control](./omni_wheel_control): Contains odometry and velocity control node scripts. These nodes when run provide topics for robot control and odometry position feedback .
- [test_base](./test_base): an example 4-wheel base using 150mm omni-wheels.

## Download and usage guide
Clone the entire repository into your local workspace with:
```
git clone https://github.com/arthurgomes4/omni_wheels_simulation.git
```
Proceed to build and source the workspace packages.

Launch gazebo with the [test_base](./test_base) model spawned along with odometry and command velocity nodes.
```
roslaunch test_base gazebo.launch
```
**NOTE**: You may have to install ros velocity controllers if they are not already present. Use command:
```
sudo apt update && sudo apt install ros-melodic-velocity-controllers
```
To control movement use:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

Additional packages:
 * gazebo-ros
 * velocity-controllers
 * controller-manager
 * joint-state-controller

[![Image](https://img.shields.io/badge/developed%20using-VSCode-green.svg)](https://code.visualstudio.com/)
[![Image](https://img.shields.io/badge/Developer-arthurgomes4-blue.svg)](https://github.com/arthurgomes4)
