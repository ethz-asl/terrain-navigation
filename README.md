# terrain-navigation

[![Build Test](https://github.com/Jaeyoung-Lim/terrain-navigation/actions/workflows/build_test.yml/badge.svg)](https://github.com/Jaeyoung-Lim/terrain-navigation/actions/workflows/build_test.yml)

This package includes a global planner based on Dubins RRT* enabling low altitude navigation for fixedwing vehicles in steep terrain.

![bitmap](https://user-images.githubusercontent.com/5248102/223867256-f2c29bd6-403a-422f-bca0-c64fc502dfe1.png)

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

## Running with PX4 SITL(Software-In-The-Loop)
To setup PX4, clone the [ETHZ ASL PX4 autopilot project](https://github.com/ethz-asl/ethzasl_fw_px4/tree/pr-hinwil-testing)
The setup instructions can be found in the [documentation](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html)

The default launch file can be run as the following command. 
```
roslaunch terrain_navigation_ros test_terrain_planner.launch
```

You can use [QGroundcontrol](http://qgroundcontrol.com/) to configure and fly the vehicle. Get the vehicle flying, and plan a mission to fly!

Once the vehicle is flying in a loiter, plan a mission, engage the planner and fly through the rviz UI

![ezgif com-video-to-gif (1)](https://user-images.githubusercontent.com/5248102/222560237-18737cb9-a1a3-43b7-8867-1fc729dc2064.gif)

