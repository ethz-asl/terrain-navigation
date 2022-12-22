#!/bin/bash

NUM_RUNS=30

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

roslaunch terrain_planner_benchmark benchmark.launch location:="dischma_valley" runs:=${NUM_RUNS}
roslaunch terrain_planner_benchmark benchmark.launch location:="mythen" runs:=${NUM_RUNS}
roslaunch terrain_planner_benchmark benchmark.launch location:="gotthard" runs:=${NUM_RUNS}
