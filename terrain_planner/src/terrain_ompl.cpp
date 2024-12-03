

#include "terrain_planner/terrain_ompl.h"

namespace ompl {

bool TerrainValidityChecker::isValid(const base::State* state) const {
  const Eigen::Vector3d position(state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getX(),
                                 state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getY(),
                                 state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getZ());
  return !checkCollision(position);
}

bool TerrainValidityChecker::checkCollision(const Eigen::Vector3d state) const {
  bool in_collision = isInCollision("distance_surface", state, true);
  bool in_collision_max{false};
  if (check_collision_max_altitude_) {
    in_collision_max = isInCollision("max_elevation", state, false);
  }
  bool valid;
  if (map_.isInside(state.head(2))) {
    valid = bool(map_.atPosition("valid", state.head(2)) > 0.5);
  }
  return in_collision | in_collision_max | !valid;
}

bool TerrainValidityChecker::isInCollision(const std::string& layer, const Eigen::Vector3d& position,
                                           bool is_above) const {
  const Eigen::Vector2d position_2d(position.x(), position.y());
  if (map_.isInside(position_2d)) {
    const double elevation = map_.atPosition(layer, position_2d);
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

}  // namespace ompl