# https://github.com/osrf/docker_images/blob/df19ab7d5993d3b78a908362cdcd1479a8e78b35/ros/noetic/ubuntu/focal/ros-core/Dockerfile
FROM ros:noetic-ros-core-focal

SHELL ["/bin/bash", "-c"]

WORKDIR /root/catkin_ws/src/terrain-navigation

COPY mav_planning_rviz/package.xml mav_planning_rviz/package.xml
COPY planner_msgs/package.xml planner_msgs/package.xml
COPY terrain_navigation/package.xml terrain_navigation/package.xml
COPY terrain_navigation_ros/package.xml terrain_navigation_ros/package.xml
COPY terrain_planner/package.xml terrain_planner/package.xml
COPY terrain_planner_benchmark/package.xml terrain_planner_benchmark/package.xml

WORKDIR /root/catkin_ws/

RUN apt-get update \
    # https://ros.org/reps/rep-2001.html#dev-tools
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        ros-dev-tools \
        ros-noetic-catkin \
        python3-catkin-tools \
        python3-pip \
    && rosdep init \
    && rosdep update \
    && DEBIAN_FRONTEND=noninteractive rosdep install --from-paths src --ignore-src -r -y \
    && python3 -m pip install \
        wstool \
    && rm -rf /var/lib/apt/lists/*
