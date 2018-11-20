# vacuum_cleaner
<p align="center">
<a href='https://opensource.org/licenses/MIT'><img src='https://img.shields.io/badge/License-MIT-brightgreen.svg'/></a>
</p>

## Overview
This project implements a simple walker algorithm for a vacuum cleaner robot with ROS and Gazebo. It has one package "vauum_cleaner" with a node "cleaner" which subscribes to the laser data of turtle bot provided by topic /scan, and publishes the command velocities for turtle bot. The launch file launches the node with the turtle bot simulation. It can also record the messages for ~30 sec using rosbag.

## Dependencies
1. ROS Kinetic - to install ROS follow the [link](http://wiki.ros.org/kinetic/Installation)
2. catkin - to install catkin run the following command
```
sudo apt-get install ros-kinetic-catkin
```
3. Package Dependencies

 a. roscpp
 
 b. turtlebot simulation stack - to install turtlebot run the following command
 ```
 sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
 ```

## Build
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone --recursive https://github.com/ajeetwankhede/vacuum_cleaner.git
cd ..
cd ..
catkin_make
```

## Launch the package
To launch the nodes using launch file run the following commands
In your catkin workspace
```
cd ~/catkin_ws
catkin_make
source ./devel/setup.bash
```
Make sure that a roscore is up and running:
```
roscore
```
To run the launch file enter the following command in a new terminal
```
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch vacuum_cleaner launch_file.launch
```

## Run the node
To run the node run the following commands
In your catkin workspace
```
cd ~/catkin_ws
catkin_make
source ./devel/setup.bash
```
Make sure that a roscore is up and running:
```
roscore
```
Make sure to launch a simple world with a Turtlebot:
```
roslaunch turtlebot_gazebo turtlebot_world.launch
```
To run the node named "cleaner" enter the following command in a new terminal
```
cd ~/catkin_ws
source ./devel/setup.bash
rosrun vacuum_cleaner cleaner
```

## Stop the nodes
To stop, press Ctrl+C and then enter the following command to clean up the nodes from the rosnode list
```
rosnode cleanup
```

## Visualize in rqt
To visualize the publish-subscribe relationships between the nodes as a graph run the following command in a new terminal
```
rosrun rqt_graph rqt_graph
```

## Using rosbag to record and replay the messages
To start recoding the messages expect /camera/* topics, using rosbag run the following commands in a new terminal. It will record for ~30 sec and dave a bag file in results folder. Make sure roscore is running.
```
roslaunch vacuum_cleaner launch_file.launch record:=true
```
The recording can be stopped by pressing Ctrl+C and the messages will be saved in results/recordedData.bag file. To disable the recording option while launching the launch_file run the following command.
```
roslaunch vacuum_cleaner launch_file.launch record:=false
```

To play the recorded messages run the following command in a new terminal.
```
cd ~/catkin_ws/src/vacuum_cleaner/results
rosbag play recordedData.bag
```
