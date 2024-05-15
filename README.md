# a_star_ros

![Build Status](https://github.com/ToshikiNakamura0412/a_star_ros/workflows/build/badge.svg)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

ROS implementation of A* search algorithm

## Environment
- Ubuntu 20.04
- ROS Noetic

## Install and Build
```
# clone repository
cd /path/to/your/catkin_ws/src
git clone https://github.com/ToshikiNakamura0412/a_star_ros.git

# build
cd /path/to/your/catkin_ws
rosdep install -riy --from-paths src --rosdistro noetic # Install dependencies
catkin build a_star_ros -DCMAKE_BUILD_TYPE=Release      # Release build is recommended
```

## How to use
```
roslaunch a_star_ros a_star.launch
```

## Running the demo
### Basic 
```
roslaunch a_star_ros test.launch
```

### planning for local map
```
# clone repository
cd /path/to/your/catkin_ws/src
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

# build
cd /path/to/your/catkin_ws
rosdep install -riy --from-paths src --rosdistro noetic
catkin build -DCMAKE_BUILD_TYPE=Release

# run demo
export TURTLEBOT3_MODEL=burger
roslaunch a_star_ros test.launch use_local_map:=true
```

## Node I/O
WIP

### Published/Subscribed Topics
WIP

## Parameters
WIP

## References
- https://myenigma.hatenablog.com/entry/20140503/1399080847
