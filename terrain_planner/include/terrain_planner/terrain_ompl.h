#ifndef TERRAIN_PLANNER_OMPL_H
#define TERRAIN_PLANNER_OMPL_H

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include "terrain_planner/DubinsAirplane2.hpp"

#include <grid_map_core/GridMap.hpp>

namespace ompl {

class TerrainValidityChecker : public base::StateValidityChecker {
 public:
  TerrainValidityChecker(const base::SpaceInformationPtr& space_info, const grid_map::GridMap& map)
      : base::StateValidityChecker(space_info), map_(map) {}

  /**
   * @brief State validity check
   *
   * @param state
   * @return true
   * @return false
   */
  virtual bool isValid(const base::State* state) const override;

  /**
   * @brief Check collisions on gridmap
   *
   * @param state
   * @return true
   * @return false
   */
  bool checkCollision(const Eigen::Vector3d state) const;

  /**
   * @brief Check if the state is in collision with a layer
   *
   * @param layer name of the layer in the elevation map
   * @param position position of the state to evaluate
   * @param is_above
   * @return true
   * @return false
   */
  bool isInCollision(const std::string& layer, const Eigen::Vector3d& position, bool is_above) const;

 protected:
  const grid_map::GridMap& map_;
};
}  // namespace ompl

#endif
