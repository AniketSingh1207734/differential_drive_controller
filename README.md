# Differential Drive Controller - ROS 2 Project

This repository contains a ROS 2 package that implements a differential drive robot controller with RPM computation for each wheel. The robot is simulated in Gazebo with navigation functionality using Nav2 and visualized in RViz.

## Setup Instructions

### 1. Import the Nav2 Task Package

First, import the Nav2 task package from the repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src

git clone https://github.com/AniketSingh1207734/Nav2_task.git
```

### 2. Build the Nav2 Task Package

Navigate to the root of your workspace and build the workspace:

```bash
cd ~/ros2_ws

colcon build
```

### 3. Import the Differential Drive Controller Package

Next, import the differential drive controller package into your workspace:

```bash
git clone https://github.com/AniketSingh1207734/differential_drive_controller.git
```

### 4. Build the Workspace

After importing both repositories, build the entire workspace again:

```bash
colcon build
```

### 5. Source the Workspace

Once the workspace is built, open a new terminal and source your workspace:

```bash
source ~/ros2_ws/install/setup.bash
source /opt/ros/humble/setup.bash
```

### 6. Launch Gazebo with the Robot

To start the simulation, launch Gazebo with the robot inside a world (e.g., `obstacle.world`):

```bash
ros2 launch my_robot launch_sim.launch.py world:=./ros2_ws/src/my_robot/worlds/obstacle.world
```

### 7. Launch the Nav2 Navigation Stack

In another terminal, source your workspace and launch the Nav2 stack:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_robot nav2_launch.launch.py
```

### 8. Run the Differential Drive Controller

Open a new terminal, source your workspace, and run the differential drive controller node:

```bash
ros2 run differential_drive_controller differential_drive_controller
```

### 9. Echo the Wheel RPMs

To monitor the wheel RPMs, open separate terminals and run the following commands:

```bash
ros2 topic echo /left_wheel_rpm
ros2 topic echo /right_wheel_rpm
```

Now you can monitor the robot's wheel speeds and see the robot navigate in both Gazebo and RViz.
