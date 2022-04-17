#ifndef TERRAIN_PLANNER_PRIMITIVE_PLANNER_H
#define TERRAIN_PLANNER_PRIMITIVE_PLANNER_H

#include "terrain_planner/maneuver_library.h"
#include "terrain_planner/planner.h"

/**
 * @brief MCTS Planner implementing the UCT algorithm
 *
 */
class PrimitivePlanner : public Planner {
 public:
  PrimitivePlanner() : Planner() { maneuver_library_ = std::make_shared<ManeuverLibrary>(); };
  virtual ~PrimitivePlanner(){};
  void setTerrainMap(std::shared_ptr<TerrainMap> map) {
    terrain_map_ = map;
    maneuver_library_->setTerrainMap(terrain_map_);
  };

  /**
   * @brief Solve MCTS with a given time budget
   *
   * @param current_pos
   * @param current_vel
   * @param current_att
   * @param current_path
   */
  virtual TrajectorySegments solve(const Eigen::Vector3d current_pos, const Eigen::Vector3d current_vel,
                                   const Eigen::Vector4d current_att, TrajectorySegments &current_path) override;

  std::vector<TrajectorySegments> getMotionPrimitives() { return tree_->getMotionPrimitives(); };

 private:
  std::shared_ptr<Primitive> tree_;
  std::shared_ptr<ManeuverLibrary> maneuver_library_;
};

#endif
