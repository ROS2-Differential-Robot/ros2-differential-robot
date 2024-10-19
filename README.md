# ROS 2 Differential Robot

## Project Overview

The **ROS 2 Differential Robot** project is designed to develop an autonomous robot capable of navigating and performing tasks within a dynamic environment. This project utilizes ROS 2 and Gazebo to simulate and implement functionalities such as obstacle avoidance, mapping and localization.

## Requirements

Before setting up the project, ensure you have the following software installed:

- **ROS 2 Jazzy**: Follow the [ROS 2 installation guide](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html).
- **Gazebo Harmonic**: Ensure that you have the Harmonic version of Gazebo installed.

## Setup Instructions

### 1. Clone the Repository

First, clone the repository to your local machine:

```bash

mkdir -p ~/adam_ws/src
cd ~/adam_ws/src
git clone https://github.com/ROS2-Differential-Robot/ros2-differential-robot.git
```

### 2. Install Dependencies

Navigate to your workspace and install the necessary dependencies using the following commands:

``` bash
cd ~/adam_ws
source /opt/ros/$ROS_DISTRO/setup.bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -i -y --rosdistro jazzy

```

### 3. Build the Project

Build the project using colcon:

```bash
colcon build
```

### 4. Source the Setup File

After the build is complete, source the setup file to overlay the workspace:

```bash
source install/setup.bash
```

### 5. Run the Simulation

You can launch the robot in a Gazebo simulation using the following command:

```bash
ros2 launch diff_drive_robot your_launch_file.launch.py
```



