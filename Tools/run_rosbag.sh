#!/bin/bash

set -e

# Manage Logs
mkdir -p /home/believer/rosbag

source $HOME/catkin_ws/devel/setup.bash
roslaunch terrain_planner run_rosbag.launch
