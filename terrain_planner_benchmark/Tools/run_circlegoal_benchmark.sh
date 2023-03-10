#!/bin/bash

NUM_RUNS=50

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
OUTPUT_PATH=${SCRIPT_DIR}/../../output

source /opt/ros/${ROS_DISTRO}/setup.bash
source ~/catkin_ws/devel/setup.bash


touch $OUTPUT_PATH/sargans.log
roslaunch terrain_planner_benchmark goaltype_benchmark.launch location:="sargans" runs:=${NUM_RUNS} > ${OUTPUT_PATH}/sargans.log
touch $OUTPUT_PATH/dischma_valley.log
roslaunch terrain_planner_benchmark goaltype_benchmark.launch location:="dischma_valley" runs:=${NUM_RUNS} > ${OUTPUT_PATH}/dischma_valley.log
touch $OUTPUT_PATH/gotthard.log
roslaunch terrain_planner_benchmark goaltype_benchmark.launch location:="gotthard" runs:=${NUM_RUNS} > ${OUTPUT_PATH}/gotthard.log
