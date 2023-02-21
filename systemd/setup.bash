#!/bin/bash

set -e

alias log_terrain_planner="journalctl -fu terrain-planner.service -n 100"
alias log_mavlink_router="journalctl -fu mavlink-router.service -n 100"
alias log_rosbag_record="journalctl -fu rosbag-record.service -n 100"
alias status_terrain_planner="systelctl status terrain-planner.service"
alias status_mavlink_router="systelctl status mavlink-router.service"
alias status_rosbag_record="systelctl status rosbag-record.service"
