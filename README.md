# terrain-navigation

[![Build Test](https://github.com/Jaeyoung-Lim/terrain-navigation/actions/workflows/build_test.yml/badge.svg)](https://github.com/Jaeyoung-Lim/terrain-navigation/actions/workflows/build_test.yml)

This package includes a global planner based on Dubins RRT* enabling low altitude navigation for fixedwing vehicles in steep terrain.

![ezgif com-gif-maker](https://user-images.githubusercontent.com/5248102/196541709-64c6a41f-9765-4978-b3fa-410912ee60e7.gif)

## Setup
Install the dependencies. This package depends on gdal, to read georeferenced images and GeoTIFF files.
```
apt install libgdal-dev
```
Configure the catkin workspace
```
catkin config --extend "/opt/ros/noetic"
catkin config --merge-devel
```

Pull in dependencies using rosinstall / rosdep
```
wstool init src src/terrain-navigation/dependencies.rosinstall
wstool update -t src -j4
rosdep update
rosdep install --from-paths src --ignore-src -y --rosdistro noetic
```

Build the package
```
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=False
catkin build -j$(nproc) -l$(nproc) terrain_navigation_ros
```

## Running the terrain planner
The planner example can be launched with the following launchfile
```
roslaunch terrain_planner test_ompl_rrt_circle.launch
```

## Running the package with PX4 SITL(Software-In-The-Loop)
To setup PX4, clone the [PX4 autopilot project](https://github.com/PX4/PX4-Autopilot)
The setup instructions can be found in the [documentation](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html)

The default launch file can be run as the following command. 
```
roslaunch terrain_navigation_ros test_terrain_planner.launch
```

Then, plan the mission with the rviz UI
![ezgif com-video-to-gif (1)](https://user-images.githubusercontent.com/5248102/222560237-18737cb9-a1a3-43b7-8867-1fc729dc2064.gif)

You can use [QGroundcontrol](http://qgroundcontrol.com/) to configure and fly the vehicle. Get the vehicle flying, and plan a mission to fly!
