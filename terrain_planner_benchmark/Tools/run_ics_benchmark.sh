#!/bin/bash

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

roslaunch terrain_planner_benchmark test_ics.launch location:="dischma_valley"
roslaunch terrain_planner_benchmark test_ics.launch location:="mythen"
roslaunch terrain_planner_benchmark test_ics.launch location:="gotthard"
