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

class TerrainStateSampler : public base::StateSampler {
 public:
  TerrainStateSampler(const ompl::base::StateSpace* si, const grid_map::GridMap& map) : StateSampler(si), map_(map) {
    // name_ = "my sampler";
  }

  void sampleUniform(ompl::base::State* state) override {
    /// TODO: We don't need to querry this everytime we sample

    const Eigen::Vector2d map_pos = map_.getPosition();
    const double map_width_x = map_.getLength().x();
    const double map_width_y = map_.getLength().y();
    double min_altitude = 50.0;
    double max_altitude = 150.0;

    double x = rng_.uniformReal(map_pos(0) - 0.5 * map_width_x, map_pos(1) + 0.5 * map_width_x);
    double y = rng_.uniformReal(map_pos(1) - 0.5 * map_width_y, map_pos(1) + 0.5 * map_width_y);
    double yaw = rng_.uniformReal(-M_PI, M_PI);

    double terrain_elevation = map_.atPosition("elevation", Eigen::Vector2d(x, y));
    double z = rng_.uniformReal(min_altitude, max_altitude) + terrain_elevation;

    state->as<fw_planning::spaces::DubinsAirplane2StateSpace::StateType>()->setX(x);
    state->as<fw_planning::spaces::DubinsAirplane2StateSpace::StateType>()->setY(y);
    state->as<fw_planning::spaces::DubinsAirplane2StateSpace::StateType>()->setZ(z);
    state->as<fw_planning::spaces::DubinsAirplane2StateSpace::StateType>()->setYaw(yaw);
    assert(si_->isValid(state));
    return;
  }
  void sampleUniformNear(ompl::base::State* /*state*/, const ompl::base::State* /*near*/,
                         const double /*distance*/) override {
    std::cout << "Sample Near" << std::endl;
    return;
  }
  void sampleGaussian(ompl::base::State* state, const ompl::base::State* mean, double stdDev) override {
    std::cout << "Sample Gaussian" << std::endl;
    return;
  }

 protected:
  ompl::RNG rng_;
  const grid_map::GridMap& map_;
};
}  // namespace ompl

#endif
