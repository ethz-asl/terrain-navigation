<img align="right" height="60" src="https://user-images.githubusercontent.com/5248102/126074528-004a32b9-7911-486a-9e79-8b78e6e66fdc.png">

# terrain-navigation

[![Build Test](https://github.com/ethz-asl/terrain-navigation/actions/workflows/build_test.yml/badge.svg)](https://github.com/ethz-asl/terrain-navigation/actions/workflows/build_test.yml)

This package includes an implementation of the RA-L submission of  "Safe Low-Altitude Navigation in Steep Terrain with Fixed-Wing Aerial Vehicles".
The implementation includes a global planner based on a RRT* in the Dubins Airplane space enabling low altitude navigation for fixed wing vehicles in steep terrain.

<p align="center">
    <img src="https://github.com/ethz-asl/terrain-navigation/assets/5248102/90e43b60-ea8c-49db-9fb3-257b145fc35c" alt="overview">
</p>

## Paper and Video
If you find this package useful in an academic context, please consider citing the paper

- Lim, Jaeyoung, Florian Achermann, Rik Girod, Nicholas Lawrance, and Roland Siegwart. "Safe Low-Altitude Navigation in Steep Terrain With Fixed-Wing Aerial Vehicles." arXiv preprint arXiv:2401.04831 (2024). [[paper](https://arxiv.org/abs/2401.04831)] [[video](https://youtu.be/7C5SsRn_L5Q?si=cMNtX16F1aFNrV8_)]

```
@article{lim2024safe,
  title={Safe Low-Altitude Navigation in Steep Terrain With Fixed-Wing Aerial Vehicles},
  author={Lim, Jaeyoung and Achermann, Florian and Girod, Rik and Lawrance, Nicholas and Siegwart, Roland},
  journal={IEEE Robotics and Automation Letters},
  year={2024},
  volume={9},
  number={5},
  pages={4599-4606},
  doi={10.1109/LRA.2024.3368800}}
```

## Setup

### Setting up the Build Environment using Docker

If your operating system doesn't support ROS 2 humble, docker is a great alternative. 

First, create the image, with the build context at the root of this repo

```Bash
docker build --file docker/Dockerfile --tag terrain-navigation-ros2 .
```

You can see the image exists:
```bash
docker images
>>> REPOSITORY                TAG       IMAGE ID       CREATED        SIZE
>>> terrain-navigation-ros2   latest    5565f845ab4f   2 weeks ago    774MB
```

Next, run the image, mounting in the source into a workspace. All the dependencies are now installed.
```Bash
docker run --network=host -it -v $(pwd):/root/ros2_ws/src/ethz-asl/terrain-navigation -w /root/ros2_ws terrain-navigation-ros2 bash
```

### Running the Build

For dependencies that do not have binaries available, pull them into your ROS workspace using vcs.
```bash
cd ~/ros2_ws/src
wget https://raw.githubusercontent.com/ethz-asl/terrain-navigation/ros2/terrain-navigation.repos
vcs import --recursive < terrain-navigation.repos
```

For dependencies available through binaries, use rosdep.
This package depends on [GDAL](https://gdal.org/index.html) to read georeferenced images and GeoTIFF files.
```bash
sudo apt update
rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src
```

Build the package
```bash
colcon build --packages-up-to terrain_navigation_ros
```

## Running with PX4 SITL(Software-In-The-Loop), Gazebo and QGroundControl (QGC)

Pre-requisites:
* Ubuntu 22.04 with ROS 2 Humble
* Gazebo harmonic

### PX4 SITL Session with Gazebo

To setup PX4, clone the a custom version of PX4.
The setup instructions can be found in the [documentation](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html) for the latest dependencies.

This branch contains a small fix to the height rate setpoint forward ported to main, and a DEM for Gazebo of the region used in the terrain planner.

First set up the PX4 development environment:
```bash
git clone https://github.com/srmainwaring/PX4-AutoPilot --branch prs/pr-hinwil-testing-rebased --recursive
cd PX4-AutoPilot
./Tools/setup/ubuntu.sh
```

Now, configure the shell environment to run `gz_standard_vtol`.
```bash
# Source environment variables mentioned in PX4-Autopilot/Tools/simulation/gz/worlds/davosdorf.sdf
export PX4_HOME_LAT=46.8132056
export PX4_HOME_LON=9.8479538
export PX4_HOME_ALT=1562.0
export PX4_GZ_VERBOSE=4
export PX4_GZ_MODEL_POSE="0,0,1562,0,0,0"
export PX4_GZ_WORLD=davosdorf
# TODO remove these variables and make it use a proper aligned world.

make px4_sitl gz_standard_vtol
```

Due to the camera spawn location bug in [PX4-Autpilot#22659](https://github.com/PX4/PX4-Autopilot/issues/22659), 
in the Gazebo Entity Tree view, click "standard_vtol_0" -> "Follow" to center the camera on the vehicle.
The QuadPlane will be visible sitting on terrain in Davos, Switzerland.

### QGC


QGroundControl (QGC) will be used for loading a mission and sending a takeoff command.
QGC can also monitor the vehicle's flight mode and status through MAVLink.

[Install QGroundControl](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html) to configure and fly the vehicle. The instructions below may be specific to version 4.3.0.

Let's get the vehicle flying, and plan a mission to fly!

1. Disconnect any flight controllers; these would interfere with SITL.
1. Start QGC.
1. [Enable virtual joystick](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/settings_view/virtual_joystick.html#enable-the-thumbsticks).    
   Without this, the QGC status bar will be yellow and report an arming check report of "No manual control input".
1. Upload a mission. Go to the "Plan" screen, click "File" -> Storage "Open..." and select the `davosdorf` mission:
   * [davosdorf_mission.plan](./terrain_navigation_ros/config/davosdorf_mission.plan)
1. Then, click the light blue flashing "Upload Required" button to upload the mission to the vehicle.
1. Go back to the "Fly" screen.
1. Use the slider to start the mission. The vehicle will launch vertically, transition to fixed wing mode, and enter a loiter next to Lake Davos.

Once the vehicle is flying in a loiter,  plan a mission, engage the planner and fly through the rviz UI (Video: https://youtu.be/EJWyGSqaKb4)

## ROS 2 Terrain Planner, MavROS and RVIZ

Launch the terrain planner, mavros, and rviz nodes in separate terminals. 

These can be combined but it helps with debugging to run them separately:

Terminal 1 - Terrain Planner:
```bash
ros2 launch terrain_navigation_ros terrain_planner.launch.py rviz:=false
```

Terminal 2 - MAVRos:
```bash
ros2 launch terrain_navigation_ros mavros.launch.py flight_stack:=px4
```

If you get warnings from mavros like this, ignore them:
```
[mavros_node-1] [WARN] [1705686179.019743422] [mavros.guided_target]: PositionTargetGlobal failed because no origin
```

TODO: Fix QOS incompatibility on the following topic: 
```bash
ros2 topic info /mavros/global_position/gp_origin -v
```


Terminal 3 - RVIZ:
```bash
ros2 launch terrain_navigation_ros rviz.launch.py
```



Set the start and goal positions. If the interactive marker server is not updating correctly this may done using service calls:

Terminal 4 - Services
```bash
ros2 service call /terrain_planner/set_start planner_msgs/srv/SetVector3 "{vector: {x: 1570, y: -330, z: -1}}"
ros2 service call /terrain_planner/set_goal planner_msgs/srv/SetVector3 "{vector: {x: -100, y: -200, z: -1}}"
```

Trigger the planner and set the planner to navigate. The equivalent ROS 2 service calls are:

Terminal 4 - Services
```bash
ros2 service call /terrain_planner/trigger_planning planner_msgs/srv/SetVector3 "{vector: {z: 10.0}}"
ros2 service call /terrain_planner/set_planner_state planner_msgs/srv/SetPlannerState "{state: 2}"
```
Figure: rviz after planning
px4_terrain_rviz_plan

Finally engage the planner by switching the plane to OFFBOARD mode. Using mavros directly this is:

ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: OFFBOARD}"
Figure: Gazebo and QGC at the goal
px4_terrain_gz_goal
px4_terrain_qgc_goal
