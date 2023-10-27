# terrain-navigation

[![Build Test](https://github.com/ethz-asl/terrain-navigation/actions/workflows/build_test.yml/badge.svg)](https://github.com/ethz-asl/terrain-navigation/actions/workflows/build_test.yml)

This package includes a global planner based on Dubins RRT* enabling low altitude navigation for fixedwing vehicles in steep terrain.

![overview3](https://github.com/ethz-asl/terrain-navigation/assets/5248102/90e43b60-ea8c-49db-9fb3-257b145fc35c)

## Setup

### Setting up the Build Environment using Docker

If your operating system doesn't support ROS 1 noetic, docker is a great alternative. 

First, create the image, with the build context at the root of this repo

```
docker build --file docker/Dockerfile --tag terrain-navigation-ros1 .
```

You can see the image exists:
```
docker images
>>> REPOSITORY                TAG       IMAGE ID       CREATED        SIZE
>>> terrain-navigation-ros1   latest    5565f845ab4f   2 weeks ago    774MB
```

Next, run the image, mounting in the source into a workspace. All the dependencies are now installed.
```
docker run --network=host -it -v $(pwd):/root/catkin_ws/src/terrain-navigation -w /root/catkin_ws terrain-navigation-ros1 bash
```

### Running the Build

Configure the catkin workspace
```
catkin config --extend "/opt/ros/noetic"
catkin config --merge-devel
```

For dependencies that do not have binaries available, pull them in using rosinstall.
```
wstool init src src/terrain-navigation/dependencies.rosinstall
wstool update -t src -j4
```

For dependencies available through binaries, use rosdep.
This package depends on [GDAL](https://gdal.org/index.html) to read georeferenced images and GeoTIFF files.
```
rosdep update
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -y
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

Once the vehicle is flying in a loiter, plan a mission, engage the planner and fly through the rviz UI (Video: https://youtu.be/EJWyGSqaKb4)

