//  June/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "terrain_planner/terrain_ompl_rrt.h"

// Constructor
TerrainOmplRrt::TerrainOmplRrt() { problem_setup_ = std::make_shared<ompl::OmplSetup>(); }
TerrainOmplRrt::~TerrainOmplRrt() {
  // Destructor
}

void TerrainOmplRrt::setupProblem() {
  problem_setup_->clear();

  problem_setup_->setDefaultPlanner();
  problem_setup_->setDefaultObjective();
  assert(map);
  problem_setup_->setOctomapCollisionChecking(map_->getGridMap());
  problem_setup_->getStateSpace()->setStateSamplerAllocator(
      std::bind(&TerrainOmplRrt::allocTerrainStateSampler, this, std::placeholders::_1));
  problem_setup_->getStateSpace()->allocStateSampler();
  ompl::base::RealVectorBounds bounds(3);
  bounds.setLow(0, lower_bound_.x());
  bounds.setLow(1, lower_bound_.y());
  bounds.setLow(2, lower_bound_.z());

  bounds.setHigh(0, upper_bound_.x());
  bounds.setHigh(1, upper_bound_.y());
  bounds.setHigh(2, upper_bound_.z());

  // Define start and goal positions.
  problem_setup_->getGeometricComponentStateSpace()->as<fw_planning::spaces::DubinsAirplane2StateSpace>()->setBounds(
      bounds);

  problem_setup_->setStateValidityCheckingResolution(0.001);

  planner_data_ = std::make_shared<ompl::base::PlannerData>(problem_setup_->getSpaceInformation());
}

void TerrainOmplRrt::setBoundsFromMap(const grid_map::GridMap& map) {
  const Eigen::Vector2d map_pos = map.getPosition();
  /// TODO: Iterate through map to get elevation bounds

  double min_elevation = std::numeric_limits<double>::max();
  double max_elevation = std::numeric_limits<double>::min();
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index map_index = *iterator;
    const double minimum_elevation_limit = map.at("distance_surface", map_index);
    if (minimum_elevation_limit < min_elevation) {
      min_elevation = minimum_elevation_limit;
    }
    const double maximum_elevation_limit = map.at("max_elevation", map_index);
    if (maximum_elevation_limit > max_elevation) {
      max_elevation = maximum_elevation_limit;
    }
  }

  const double map_width_x = map.getLength().x();
  const double map_width_y = map.getLength().y();
  double roi_ratio = 0.5;
  Eigen::Vector3d lower_bounds{
      Eigen::Vector3d(-map_pos(0) - roi_ratio * map_width_x, map_pos(1) - roi_ratio * map_width_y, min_elevation)};
  Eigen::Vector3d upper_bounds{
      Eigen::Vector3d(map_pos(0) + roi_ratio * map_width_x, map_pos(1) + roi_ratio * map_width_y, max_elevation)};
  std::cout << "[TerrainOmplRrt] Upper bounds: " << upper_bounds.transpose() << std::endl;
  std::cout << "[TerrainOmplRrt] Lower bounds: " << lower_bounds.transpose() << std::endl;
  setBounds(lower_bounds, upper_bounds);
}

bool TerrainOmplRrt::solve(const double time_budget, const Eigen::Vector3d& start, const Eigen::Vector3d& goal,
                           std::vector<Eigen::Vector3d>& path) {
  std::cout << "[TerrainOmplRrt] start solve" << std::endl;
  path.clear();

  ompl::base::ScopedState<fw_planning::spaces::DubinsAirplane2StateSpace> start_ompl(
      problem_setup_->getSpaceInformation());
  ompl::base::ScopedState<fw_planning::spaces::DubinsAirplane2StateSpace> goal_ompl(
      problem_setup_->getSpaceInformation());

  start_ompl->setX(start.x());
  start_ompl->setY(start.y());
  start_ompl->setZ(start.z());

  goal_ompl->setX(goal.x());
  goal_ompl->setY(goal.y());
  goal_ompl->setZ(goal.z());
  std::cout << "[TerrainOmplRrt] Problem Setup" << std::endl;
  problem_setup_->setStartAndGoalStates(start_ompl, goal_ompl);
  std::cout << "[TerrainOmplRrt] setStartAndGoalStates" << std::endl;
  problem_setup_->setup();
  std::cout << "[TerrainOmplRrt] Solve" << std::endl;
  if (problem_setup_->solve(time_budget)) {
    std::cout << "Found solution:" << std::endl;
    // problem_setup_.getSolutionPath().print(std::cout);
    // problem_setup_.simplifySolution();
    problem_setup_->getSolutionPath().print(std::cout);

    problem_setup_->getPlannerData(*planner_data_);
    solve_duration_ = problem_setup_->getLastPlanComputationTime();

  } else {
    std::cout << "Solution Not found" << std::endl;
  }

  if (problem_setup_->haveSolutionPath()) {
    solutionPathToTrajectoryPoints(problem_setup_->getSolutionPath(), path);
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
    trajectory_points.emplace_back(
        Eigen::Vector3d(state_ptr->as<fw_planning::spaces::DubinsAirplane2StateSpace::StateType>()->getX(),
                        state_ptr->as<fw_planning::spaces::DubinsAirplane2StateSpace::StateType>()->getY(),
                        state_ptr->as<fw_planning::spaces::DubinsAirplane2StateSpace::StateType>()->getZ()));
  }
}
