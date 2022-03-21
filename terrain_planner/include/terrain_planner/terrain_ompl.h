#ifndef TERRAIN_OMPL_H
#define TERRAIN_OMPL_H

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include "terrain_planner/DubinsAirplane2.hpp"

#include <grid_map_core/GridMap.hpp>

namespace ompl {

class TerrainValidityChecker : public base::StateValidityChecker {
 public:
  TerrainValidityChecker(const base::SpaceInformationPtr& space_info, const grid_map::GridMap& map)
      : base::StateValidityChecker(space_info), map_(map) {}
  virtual bool isValid(const base::State* state) const {
    Eigen::Vector3d position(state->as<fw_planning::spaces::DubinsAirplane2StateSpace::StateType>()->getX(),
                             state->as<fw_planning::spaces::DubinsAirplane2StateSpace::StateType>()->getY(),
                             state->as<fw_planning::spaces::DubinsAirplane2StateSpace::StateType>()->getZ());
    bool collision = checkCollision(position);
    if (collision)
      return false;
    else
      return true;
  }

  virtual bool checkCollision(const Eigen::Vector3d state) const {
    bool is_collision = isInCollision("distance_surface", state, true) | isInCollision("max_elevation", state, false);
    return is_collision;
  }
  virtual bool isInCollision(const std::string& layer, const Eigen::Vector3d& position, bool is_above) const {
    Eigen::Vector2d position_2d(position(0), position(1));
    if (map_.isInside(position_2d)) {
      double elevation = map_.atPosition(layer, position_2d);
      if (is_above) {
        if (elevation > position(2))
          return true;
        else
          return false;
      } else {
        if (elevation < position(2))
          return true;
        else
          return false;
      }
    } else {
      return true;  // Do not allow vehicle to go outside the map
    }
  }

 protected:
  const grid_map::GridMap& map_;
};
}  // namespace ompl

#endif
