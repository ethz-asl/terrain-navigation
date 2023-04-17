#!/bin/bash

NUM_RUNS=50

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
OUTPUT_PATH=${SCRIPT_DIR}/../../output

source /opt/ros/${ROS_DISTRO}/setup.bash
source ~/catkin_ws/devel/setup.bash


roslaunch terrain_planner_benchmark test_dubins_classification.launch

python3 ${SCRIPT_DIR}/benchmark_dubins_classification.py
