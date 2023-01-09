#!/bin/bash

NUM_RUNS=10

source /opt/ros/${ROS_DISTRO}/setup.bash
source ~/catkin_ws/devel/setup.bash

roslaunch terrain_planner_benchmark benchmark.launch location:="sargans" runs:=${NUM_RUNS}
roslaunch terrain_planner_benchmark benchmark.launch location:="dischma_valley" runs:=${NUM_RUNS}
roslaunch terrain_planner_benchmark benchmark.launch location:="gotthard" runs:=${NUM_RUNS}
