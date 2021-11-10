#!/bin/bash

set -e

source $HOME/catkin_ws/devel/setup.bash
roslaunch terrain_planner run_planner.launch
