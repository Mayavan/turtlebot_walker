# turtlebot_walker
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Overview
This package is a demonstration of the walker algorithm which changes direction when it encounters an obstacle. The demo shows the turtle bot changing direction in the turtlebot3 world when an obstacle is nearby.

The launch file accepts argument to record or not record the topics using rosbag. (it does not record then camera topics to save data space)
The rosbag file is generated in the results directory.

## Dependencies
This package has been tested on a device running Ubuntu 16.04 and ROS Kinetic Kame.

To install ROS Kinetic Kame in Ubuntu 16.04, follow the steps in this [link](http://wiki.ros.org/kinetic/Installation/Ubuntu).

To install catkin, follow the installation steps in this [link](http://wiki.ros.org/catkin).

## Standard install via command-line

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
$ cd src/
$ git clone https://github.com/Mayavan/turtlebot_walker.git
$ cd ..
$ catkin_make
$ source devel/setup.bash
```


## To run demo

### Run using launch file

```
roslaunch turtlebot_walker walker.launch
```

Run using launch file and record all topics except camera with rosbag.

```
roslaunch turtlebot_walker walker.launch record:=true
```

### Run calling individual nodes

To start ROS master:

```
$ roscore
```

To run walker node:

```
$ rosrun turtlebot_walker walker
```

## To inspect rosbag file
In the Results folder

```
rosbag info ROSBAG_2018-11-20-11-45-56.bag
```

## How to play a rosbag file  
Start rosmaster node

```
roscore
```

Open a new terminal window.Go to the Results folder

```
rosbag play ROSBAG_2018-11-20-11-45-56.bag
```
Note: Gazebo should not be running