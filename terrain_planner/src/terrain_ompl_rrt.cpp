//  June/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "terrain_planner/terrain_ompl_rrt.h"

using namespace Eigen;
using namespace std;
// Constructor
TerrainOmplRrt::TerrainOmplRrt(grid_map::GridMap& map) : map_(map) {}
TerrainOmplRrt::~TerrainOmplRrt() {
  // Destructor
}

void TerrainOmplRrt::setupProblem() {
  problem_setup_.setDefaultPlanner();
  problem_setup_.setDefaultObjective();
  problem_setup_.setOctomapCollisionChecking(map_);
  ompl::base::RealVectorBounds bounds(3);
  bounds.setLow(0, lower_bound_.x());
  bounds.setLow(1, lower_bound_.y());
  bounds.setLow(2, lower_bound_.z());

  bounds.setHigh(0, upper_bound_.x());
  bounds.setHigh(1, upper_bound_.y());
  bounds.setHigh(2, upper_bound_.z());

  // Define start and goal positions.
  problem_setup_.getGeometricComponentStateSpace()->as<fw_planning::spaces::DubinsAirplane2StateSpace>()->setBounds(
      bounds);

  problem_setup_.setStateValidityCheckingResolution(0.001);

  planner_data_ = std::make_shared<ompl::base::PlannerData>(problem_setup_.getSpaceInformation());
}

bool TerrainOmplRrt::Solve(const Eigen::Vector3d& start, const Eigen::Vector3d& goal,
                           std::vector<Eigen::Vector3d>& path) {
  std::cout << "[TerrainOmplRrt] start solve" << std::endl;
  problem_setup_.clear();
  path.clear();

  ompl::base::ScopedState<fw_planning::spaces::DubinsAirplane2StateSpace> start_ompl(
      problem_setup_.getSpaceInformation());
  ompl::base::ScopedState<fw_planning::spaces::DubinsAirplane2StateSpace> goal_ompl(
      problem_setup_.getSpaceInformation());

  start_ompl->setX(start(0));
  start_ompl->setY(start(1));
  start_ompl->setZ(start(2));

  goal_ompl->setX(goal(0));
  goal_ompl->setY(goal(1));
  goal_ompl->setZ(goal(2));
  std::cout << "[TerrainOmplRrt] Problem Setup" << std::endl;
  problem_setup_.setStartAndGoalStates(start_ompl, goal_ompl);
  std::cout << "[TerrainOmplRrt] setStartAndGoalStates" << std::endl;
  problem_setup_.setup();
  std::cout << "[TerrainOmplRrt] Solve" << std::endl;
  if (problem_setup_.solve(10.0)) {
    std::cout << "Found solution:" << std::endl;
    // problem_setup_.getSolutionPath().print(std::cout);
    // problem_setup_.simplifySolution();
    problem_setup_.getSolutionPath().print(std::cout);

    problem_setup_.getPlannerData(*planner_data_);

  } else {
    std::cout << "Solution Not found" << std::endl;
  }

  if (problem_setup_.haveSolutionPath()) {
    solutionPathToTrajectoryPoints(problem_setup_.getSolutionPath(), path);
    return true;
  }
  return false;
}
void TerrainOmplRrt::solutionPathToTrajectoryPoints(ompl::geometric::PathGeometric& path,
                                                    std::vector<Eigen::Vector3d>& trajectory_points) const {
  trajectory_points.clear();
  path.interpolate();
  // trajectory_points->reserve(path.getStateCount());

  std::vector<ompl::base::State*>& state_vector = path.getStates();

  for (ompl::base::State* state_ptr : state_vector) {
    Eigen::Vector3d position(state_ptr->as<fw_planning::spaces::DubinsAirplane2StateSpace::StateType>()->getX(),
                             state_ptr->as<fw_planning::spaces::DubinsAirplane2StateSpace::StateType>()->getY(),
                             state_ptr->as<fw_planning::spaces::DubinsAirplane2StateSpace::StateType>()->getZ());

    trajectory_points.emplace_back(position);
  }
}
