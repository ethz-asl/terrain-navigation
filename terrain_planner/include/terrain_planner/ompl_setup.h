#ifndef TERRAIN_PLANNER_OMPL_SETUP_H_
#define TERRAIN_PLANNER_OMPL_SETUP_H_

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include "ompl/base/SpaceInformation.h"
#include "terrain_planner/DubinsAirplane.hpp"
#include "terrain_planner/terrain_ompl.h"

enum PlannerType { RRTSTAR, INFORMED_RRTSTAR, RRTCONNECT, BITSTAR, FMTSTAR };

namespace ompl {

class OmplSetup : public geometric::SimpleSetup {
 public:
  OmplSetup(const base::StateSpacePtr& space_ptr) : geometric::SimpleSetup(space_ptr) {}

  void setDefaultObjective() {
    setOptimizationObjective(
        ompl::base::OptimizationObjectivePtr(new ompl::base::PathLengthOptimizationObjective(getSpaceInformation())));
  }

  void setDefaultPlanner(PlannerType planner_type = PlannerType::RRTSTAR) {
    switch (planner_type) {
      case PlannerType::RRTSTAR: {
        auto planner = std::make_shared<ompl::geometric::RRTstar>(getSpaceInformation());
        planner->setRange(600.0);
        // planner->setGoalBias(goal_bias);
        setPlanner(planner);
        break;
      }
      case PlannerType::RRTCONNECT: {
        auto planner = std::make_shared<ompl::geometric::RRTConnect>(getSpaceInformation());
        planner->setRange(600.0);
        // planner->setGoalBias(goal_bias);
        setPlanner(planner);
        break;
      }
      case PlannerType::INFORMED_RRTSTAR: {
        auto planner = std::make_shared<ompl::geometric::InformedRRTstar>(getSpaceInformation());
        // planner->setRange(600.0);
        // planner->setGoalBias(goal_bias);
        setPlanner(planner);
        break;
      }
      case PlannerType::BITSTAR: {
        auto planner = std::make_shared<ompl::geometric::BITstar>(getSpaceInformation());
        setPlanner(planner);
        break;
      }
      case PlannerType::FMTSTAR: {
        auto planner = std::make_shared<ompl::geometric::FMT>(getSpaceInformation());
        setPlanner(planner);
        break;
      }
    }
  }

  const base::StateSpacePtr& getGeometricComponentStateSpace() const { return getStateSpace(); }

  void setStateValidityCheckingResolution(double resolution) {
    // This is a protected attribute, so need to wrap this function.
    si_->setStateValidityCheckingResolution(resolution);
  }

  void setTerrainCollisionChecking(const grid_map::GridMap& map, bool check_max_altitude) {
    std::shared_ptr<TerrainValidityChecker> validity_checker(
        new TerrainValidityChecker(getSpaceInformation(), map, check_max_altitude));

    setStateValidityChecker(base::StateValidityCheckerPtr(validity_checker));
  }
};

}  // namespace ompl

#endif
