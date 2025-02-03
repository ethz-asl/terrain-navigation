# Terrain Planner Benchmark

This package currently depends on a private terrain dataset.
See [74](https://github.com/ethz-asl/terrain-navigation/issues/74) for details.

## Build

Build this repository, and your terrain source.
```
colcon build --packages-up-to terrain_planner_benchmark terrain_models
```

## Terrain Planner Benchmark Node

```bash
source install/setup.bash
ros2 launch terrain_planner_benchmark goaltype_benchmark.launch
```
