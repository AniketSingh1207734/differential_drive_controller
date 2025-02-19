Differential Drive Robot Controller

This repository contains a ROS 2 package for controlling a differential drive robot. It integrates with the ROS 2 navigation stack and implements RPM computation for each wheel. Follow the instructions below to set up the workspace, build the packages, and run the simulation and controller.

Prerequisites

ROS 2 (Foxy, Galactic, or Humble)

Colcon build tool

Installation and Setup

Step 1: Clone and Build the Navigation Package

Clone the Nav2 Task repository into your ROS 2 workspace:

cd ~/ros2_ws/src
git clone https://github.com/AniketSingh1207734/Nav2_task.git

Build the workspace:

cd ~/ros2_ws
colcon build

Step 2: Clone and Build the Differential Drive Controller

Clone the Differential Drive Controller repository into your ROS 2 workspace:

cd ~/ros2_ws/src
git clone https://github.com/AniketSingh1207734/differential_drive_controller.git

Build the workspace again to include the differential drive controller:

cd ~/ros2_ws
colcon build

Step 3: Source the Workspace

Before running any commands, ensure you source the workspace in every new terminal session:

source ~/ros2_ws/install/setup.bash

Running the Simulation and Controller

Step 4: Launch the Simulation

Open a new terminal, source the workspace, and launch the robot simulation in a world with obstacles:

ros2 launch my_robot launch_sim.launch.py world:=./ros2_ws/src/my_robot/worlds/obstacle.world

Step 5: Launch the Navigation Stack

In a separate terminal, source the workspace, and launch the ROS 2 navigation stack:

ros2 launch my_robot nav2_launch.launch.py

Step 6: Run the Differential Drive Controller

In another terminal, source the workspace, and run the differential drive controller:

ros2 run differential_drive_controller differential_drive_controller

Step 7: Monitor Wheel RPM Topics

In separate terminals, monitor the RPM values of the left and right wheels using the following commands:

ros2 topic echo /left_wheel_rpm

ros2 topic echo /right_wheel_rpm

Summary

This setup provides a differential drive robot simulation with navigation and RPM monitoring. You can modify the controller logic, navigation parameters, and world configuration for further experimentation.
