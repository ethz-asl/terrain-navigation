#!/bin/bash

set -e

ROS_MASTER_URI=http://172.30.57.2:11311
alias believer_ssh="ssh ethzasl@172.30.57.2"
alias gcs_rviz="roslaunch mav_planning_rviz run_rviz.launch"