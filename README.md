# EV Charger Robot

## Introduction

EV Charging Bot based on the FANUC CRX-10iA/L cobot that can dock to a charger using a Stereo camera based
perception system.

The perception system generates a goal in 3D space from a video feed and an velocity IK based solver designed from scratch
generates and executes a real-time trajectory using the SRI Jacobian inverse method using Damped Least Squares.

**Simulation** (check /media)

![gif of implementation](./media/sim.gif)


### Dependencies
This project makes use of the ROS Galactic Geochelone distribution and is assumed to be a dependency. <br>
Find installation instructions [here](https://docs.ros.org/en/galacticInstallation.html)

### Setup / Build Instructions

```bash
$ source /opt/ros/galactic/setup.bash
# Make your ros2 workspace
$ mkdir -p ~/ros_ws/src
# Go to the source directory of your ros2 workspace
$ cd ~/ros_ws/src
#Clone the repository
$ git clone git@github.com:vinay-lanka/car_charger_robot.git
#Go back to the ws directory
$ cd ~/ros_ws
# Install rosdep dependencies before building the package
$ rosdep install -i --from-path src --rosdistro galactic -y
# Build the package using colcon build
$ colcon build --packages-select crx_description
```

### Usage

To launch the world with the arm and the 3D Camera, open a terminal and run
```bash
$ source /opt/ros/galactic/setup.bash
$ cd ~/ros_ws
$ source ./install/setup.bash
# Run the publisher in terminal
$ ros2 launch crx_description car_world.launch.py
```
To run the perception node open up another terminal and run
```bash
$ source /opt/ros/galactic/setup.bash
$ cd ~/ros_ws
$ source ./install/setup.bash
# Run the publisher in terminal
$ ros2 run crx_description perception_node.py
```
To run the trajectory publisher node open up another terminal and run
```bash
$ source /opt/ros/galactic/setup.bash
$ cd ~/ros_ws
$ source ./install/setup.bash
# Run the publisher in terminal
$ ros2 run crx_description ik_publisher.py
```
