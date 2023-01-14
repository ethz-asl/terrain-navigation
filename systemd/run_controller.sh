#!/bin/bash

set -e

source $HOME/catkin_ws/devel/setup.bash
roslaunch terrain_navigation_ros run_planner.launch
