#ifndef TERRAIN_PLANNER_PLANNER_H
#define TERRAIN_PLANNER_PLANNER_H

#include <Eigen/Dense>

#include <terrain_navigation/terrain_map.h>
#include <terrain_navigation/trajectory.h>

/**
 * @brief Planner base class
 *
 */
class Planner {
 public:
  Planner(){};
  virtual ~Planner(){};
  void setTerrainMap(std::shared_ptr<TerrainMap> map) { terrain_map_ = map; };
  virtual TrajectorySegments solve(const Eigen::Vector3d current_pos, const Eigen::Vector3d current_vel,
                                   const Eigen::Vector4d current_att, TrajectorySegments &current_path) = 0;
  void setGoal(const Eigen::Vector3d goal) { goal_ = goal; };

 protected:
  std::shared_ptr<TerrainMap> terrain_map_;
  Eigen::Vector3d goal_{Eigen::Vector3d::Zero()};

 private:
};

#endif
