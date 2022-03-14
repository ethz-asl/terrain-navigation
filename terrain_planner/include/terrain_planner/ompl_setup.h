#ifndef OMPL_SETUP_H
#define OMPL_SETUP_H

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include "ompl/base/SpaceInformation.h"

#include <terrain_planner/terrain_ompl.h>

namespace ompl {

class OmplSetup : public geometric::SimpleSetup {
 public:
  OmplSetup() : geometric::SimpleSetup(base::StateSpacePtr(new base::RealVectorStateSpace(3))) {}

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

  void setOctomapCollisionChecking(grid_map::GridMap& map) {
    std::shared_ptr<TerrainValidityChecker> validity_checker(new TerrainValidityChecker(getSpaceInformation(), map));

    setStateValidityChecker(base::StateValidityCheckerPtr(validity_checker));
  }
};

}  // namespace ompl

#endif
