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

## Running the terrain planner
The planner example can be launched with the following launchfile
```
roslaunch terrain_planner test_ompl_rrt_circle.launch
```

## Running with PX4 SITL(Software-In-The-Loop)
To setup PX4, clone the [PX4 autopilot project](https://github.com/PX4/PX4-Autopilot)
The setup instructions can be found in the [documentation](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html)

The default launch file can be run as the following command. 
```
roslaunch terrain_navigation_ros test_terrain_planner.launch
```

Then, plan the mission with the rviz UI
![ezgif com-video-to-gif (1)](https://user-images.githubusercontent.com/5248102/222560237-18737cb9-a1a3-43b7-8867-1fc729dc2064.gif)

You can use [QGroundcontrol](http://qgroundcontrol.com/) to configure and fly the vehicle. Get the vehicle flying, and plan a mission to fly!

## Running planner benchmarks
Build the terrain planner benchmarking package
```
catkin build terrain_planner_benchmark
```
There are two planner benchmarks available
  - Circle goal type benchmark
  - Planner performance benchmark (OMPL benchmarking framework)
  - Valid goal state coverage

### Goal type Benchmark
The circle goal type benchmark runs multiple planning instances to evaluate the performance differences between using a circular goal state or a yaw goal state
```
roslaunch terrain_planner_benchmark goaltype_benchmark.launch
```
### OMPL planner performance Benchmark
The OMPL planner performance benchmark uses the [OMPL benchmarking framework](https://ompl.kavrakilab.org/benchmark.html) to benchmark different planner configurations.
```
roslaunch terrain_planner_benchmark ompl_benchmark.launch [location:=gotthard] [runs:=50] [visualization:=false]
```
The benchmark results will get outputted as a file `output.log` in the `output` directory of the root of this repository.
To visualize the benchmarks, you can run the following python script
```
python3 terrain_planner_benchmark/Tools/visualize_ompl_benchmark.py -p output/output.log
```

![Best_Cost](https://user-images.githubusercontent.com/5248102/224292180-d20e4e08-9d06-405c-83f2-774e2892a0d4.png)

###  Valid goal state coverage
Calculates the proportion of valid goal state with a given radius for the terminal loiter circle
```
roslaunch terrain_planner_benchmark test_ics.launch [location:=sertig] [visualization:=true]
```
