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
wstool init src src/adaptive-mapping/dependencies.rosinstall
wstool update -t src -j4
rosdep update
rosdep install --from-paths src --ignore-src -y --rosdistro noetic
```

Build the package
```
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=False
catkin build -j$(nproc) -l$(nproc) terrain_planner
```
## Running the package
The default launch file can be run as the following command. 
```
roslaunch terrain_planner test_terrain_planner.launch
```
