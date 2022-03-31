#ifndef TERRAIN_PLANNER_OMPL_SETUP_H_
#define TERRAIN_PLANNER_OMPL_SETUP_H_

#include "terrain_planner/DubinsAirplane2.hpp"
#include "terrain_planner/terrain_ompl.h"

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include "ompl/base/SpaceInformation.h"

namespace ompl {

class OmplSetup : public geometric::SimpleSetup {
 public:
  OmplSetup() : geometric::SimpleSetup(base::StateSpacePtr(new fw_planning::spaces::DubinsAirplane2StateSpace())) {}

  void setDefaultObjective() {
    getProblemDefinition()->setOptimizationObjective(
        ompl::base::OptimizationObjectivePtr(new ompl::base::PathLengthOptimizationObjective(getSpaceInformation())));
  }

  void setDefaultPlanner() { setRrtStar(); }

  void setRrtStar() { setPlanner(ompl::base::PlannerPtr(new ompl::geometric::RRTstar(getSpaceInformation()))); }

  const base::StateSpacePtr& getGeometricComponentStateSpace() const { return getStateSpace(); }

  void setStateValidityCheckingResolution(double resolution) {
    // This is a protected attribute, so need to wrap this function.
    si_->setStateValidityCheckingResolution(resolution);
  }

  void setOctomapCollisionChecking(const grid_map::GridMap& map) {
    std::shared_ptr<TerrainValidityChecker> validity_checker(new TerrainValidityChecker(getSpaceInformation(), map));

    setStateValidityChecker(base::StateValidityCheckerPtr(validity_checker));
  }
};

}  // namespace ompl

#endif
