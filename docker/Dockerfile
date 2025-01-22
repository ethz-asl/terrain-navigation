# https://github.com/osrf/docker_images/blob/27cc0b68263bbbb10bb58dd814efc0a6b0a01ec7/ros/humble/ubuntu/jammy/ros-core/Dockerfile
FROM ros:humble-ros-core-jammy AS repo-deps
# This layer installs basic tools and the direct dependencies of terrain-navigation.

SHELL ["/bin/bash", "-c"]

WORKDIR /root/ros2_ws/src/ethz-asl/terrain-navigation

COPY mav_planning_rviz/package.xml mav_planning_rviz/package.xml
COPY planner_msgs/package.xml planner_msgs/package.xml
COPY terrain_navigation/package.xml terrain_navigation/package.xml
COPY terrain_navigation_ros/package.xml terrain_navigation_ros/package.xml
COPY terrain_planner/package.xml terrain_planner/package.xml
COPY terrain_planner_benchmark/package.xml terrain_planner_benchmark/package.xml

WORKDIR /root/ros2_ws/

RUN apt-get update \
    # https://ros.org/reps/rep-2001.html#dev-tools
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        ros-dev-tools \
        python3-pip \
        python3-colcon-mixin \
    && colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml \
    && colcon mixin update default \
    && rosdep init \
    && rosdep update \
    && DEBIAN_FRONTEND=noninteractive rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*


FROM repo-deps AS all-deps
# This layer installs dependencies of the other source packages.

COPY terrain-navigation.repos src/ethz-asl/terrain-navigation/terrain-navigation.repos

WORKDIR /root/ros2_ws/src
RUN vcs import --recursive --debug --skip-existing < ethz-asl/terrain-navigation/terrain-navigation.repos

WORKDIR /root/ros2_ws/
RUN apt-get update \
    && rosdep update \
    && source /opt/ros/humble/setup.bash \
    && rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

FROM all-deps AS build

WORKDIR /root/ros2_ws/src/ethz-asl/terrain-navigation
COPY . .
WORKDIR /root/ros2_ws/

RUN source /opt/ros/humble/setup.bash \
    && colcon build --mixin release
