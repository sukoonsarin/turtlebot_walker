[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
# turtlebot_walker

## Overview

The turtlebot_walker package implements a walker algorithm for the Turtlebot 3 in Gazebo. This ROS package is a simple turtlebot navigator that detects obstacles and avoids them by rotating and moving forward, just like the Roomba Robot!
<p align="center">
 <img src="https://github.com/sukoonsarin/turtlebot_walker/blob/main/Results/Simulation.gif">
</p>

## Dependencies

The following dependencies are required to run this package:

1. ROS Melodic
2. catkin 
3. Ubuntu 18.04 
4. Gazebo
5. ROS package for turtlebot3

To install turtlebot3 ROS package as a binary package on your system, open your terminal and run the following command :
```
sudo apt-get install ros-melodic-turtlebot3-*
```
Next for running Turtlebot3 simulation, you have to specify the model name of TurtleBot3.

Add the following command to the end of bashrc file:
```
export TURTLEBOT3_MODEL=burger
```
**To install Gazebo**

    curl -sSL http://get.gazebosim.org | sh
    gazebo


## Standard install via command-line
```
cd <<Your_catkin_workspace>>/src
git clone --recursive https://github.com/sukoonsarin/turtlebot_walker
cd ../..
catkin_make
source devel/setup.bash
```
## Running Simulation
Open the following terminals and run the following commands in them:

**1. Terminal 1:**

    cd <<Your_catkin_workspace>>
    source devel/setup.bash
    roslaunch turtlebot_walker walker.launch 

**2. Terminal 2:**

After running the Publisher/Subscriber node,we can view the full list of topics that are currently being published in the running system by running this in a new terminal:

    rostopic list -v

Press Ctrl+C in terminal 1 to terminate Gazebo simulation.

## Recording rosbag file
To enable/disable recording of bag file, we can pass the argumanet rosbagEnable:=true in the roslaunch command:
```
roslaunch turtlebot_walker walker.launch rosbagEnable:=true
```
Press Ctrl+C to terminate

## Inspecting rosbag file
To inspect the .bag file, in a terminal type:
```
cd <<Your_catkin_workspace>>
source devel/setup.bash
cd src/turtlebot_walker/Results/
rosbag info walkertopics.bag
```

## Playing the rosbag file
**1. Terminal 1:**
run master node

    roscore


**2. Terminal 2:**
play rosbag

    cd <<Your_catkin_workspace>>
    source devel/setup.bash
    cd src/turtlebot_walker/Results/
    rosbag play walkertopics.bag

Note: During rosbag playback, Gazebo should'nt be running.