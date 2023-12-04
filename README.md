<img align="right" height="60" src="https://user-images.githubusercontent.com/5248102/126074528-004a32b9-7911-486a-9e79-8b78e6e66fdc.png">

# terrain-navigation

[![Build Test](https://github.com/ethz-asl/terrain-navigation/actions/workflows/build_test.yml/badge.svg)](https://github.com/ethz-asl/terrain-navigation/actions/workflows/build_test.yml)

This package includes an implementation of the RA-L submission of  "Safe Low-Altitude Navigation in Steep Terrain with Fixed-Wing Aerial Vehicles".
The implementation includes a global planner based on a RRT* in the Dubins Airplane space enabling low altitude navigation for fixed wing vehicles in steep terrain.

<p align="center">
    <img src="https://github.com/ethz-asl/terrain-navigation/assets/5248102/90e43b60-ea8c-49db-9fb3-257b145fc35c" alt="overview">
</p>

## Setup

### Setting up the Build Environment using Docker

If your operating system doesn't support ROS 1 noetic, docker is a great alternative. 

First, create the image, with the build context at the root of this repo

```Bash
docker build --file docker/Dockerfile --tag terrain-navigation-ros1 .
```

You can see the image exists:
```Bash
docker images
>>> REPOSITORY                TAG       IMAGE ID       CREATED        SIZE
>>> terrain-navigation-ros1   latest    5565f845ab4f   2 weeks ago    774MB
```

Next, run the image, mounting in the source into a workspace. All the dependencies are now installed.
```Bash
docker run --network=host -it -v $(pwd):/root/catkin_ws/src/terrain-navigation -w /root/catkin_ws terrain-navigation-ros1 bash
```

### Running the Build

Configure the catkin workspace
```Bash
catkin config --extend "/opt/ros/noetic"
catkin config --merge-devel
```

For dependencies that do not have binaries available, pull them into your ROS workspace using vcs.
```Bash
cd ~/ros2_ws/src
wget https://raw.githubusercontent.com/ethz-asl/terrain-navigation/ros2/terrain-navigation.repos
vcs import --recursive < terrain-navigation.repos
```

For dependencies available through binaries, use rosdep.
This package depends on [GDAL](https://gdal.org/index.html) to read georeferenced images and GeoTIFF files.
```Bash
sudo apt update
rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src
```

Build the package
```Bash
colcon build --packages-up-to terrain_navigation_ros
```

## Running with PX4 SITL(Software-In-The-Loop)

To setup PX4, clone the [ETHZ ASL PX4 autopilot project](https://github.com/ethz-asl/ethzasl_fw_px4/tree/pr-hinwil-testing)
The setup instructions can be found in the [documentation](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html) for the latest dependencies.

For Ubuntu 20.04+ROS noetic with Gazebo classic:

```Bash
git clone https://github.com/ethz-asl/ethzasl_fw_px4.git --branch pr-hinwil-testing --recursive
cd ..
cd ethzasl_fw_px4
bash ./Tools/setup/ubuntu.sh

make px4_sitl gazebo
```

Do source the relevant environment variables before launching terrain navigation. For instance, for Ubuntu 20.04 + ROS Noetic + Gazebo classic:

```Bash
px4_dir=~/ethzasl_fw_px4
source $px4_dir/Tools/simulation/gazebo/setup_gazebo.bash $px4_dir $px4_dir/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$px4_dir
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$px4_dir/Tools/simulation/gazebo/sitl_gazebo
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins
```

*Note*: The path for gazebo may vary for a different version of Gazebo, update `GAZEBO_PLUGIN_PATH` as well as `ROS_PACKAGE_PATH`
accordingly.


The default launch file can be run as the following command.
```Bash
roslaunch terrain_navigation_ros test_terrain_planner.launch
```

You can use [QGroundcontrol](http://qgroundcontrol.com/) to configure and fly the vehicle. Get the vehicle flying, and plan a mission to fly!

Once the vehicle is flying in a loiter, plan a mission, engage the planner and fly through the rviz UI (Video: https://youtu.be/EJWyGSqaKb4)

