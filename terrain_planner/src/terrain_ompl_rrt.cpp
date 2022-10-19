//  June/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "terrain_planner/terrain_ompl_rrt.h"

// Constructor
TerrainOmplRrt::TerrainOmplRrt() { problem_setup_ = std::make_shared<ompl::OmplSetup>(); }
TerrainOmplRrt::~TerrainOmplRrt() {
  // Destructor
}

void TerrainOmplRrt::setupProblem(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel,
                                  const Eigen::Vector3d& goal) {
  problem_setup_->clear();

  problem_setup_->setDefaultPlanner();
  problem_setup_->setDefaultObjective();
  assert(map);
  problem_setup_->setTerrainCollisionChecking(map_->getGridMap());
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
  problem_setup_->getGeometricComponentStateSpace()->as<fw_planning::spaces::DubinsAirplaneStateSpace>()->setBounds(
      bounds);

  problem_setup_->setStateValidityCheckingResolution(0.001);

  planner_data_ = std::make_shared<ompl::base::PlannerData>(problem_setup_->getSpaceInformation());

  ompl::base::ScopedState<fw_planning::spaces::DubinsAirplaneStateSpace> start_ompl(
      problem_setup_->getSpaceInformation());
  ompl::base::ScopedState<fw_planning::spaces::DubinsAirplaneStateSpace> goal_ompl(
      problem_setup_->getSpaceInformation());

  start_ompl->setX(start_pos(0));
  start_ompl->setY(start_pos(1));
  start_ompl->setZ(start_pos(2));
  double start_yaw = std::atan2(start_vel(1), start_vel(0));
  start_ompl->setYaw(start_yaw);

  goal_ompl->setX(goal(0));
  goal_ompl->setY(goal(1));
  goal_ompl->setZ(goal(2));
  problem_setup_->setStartAndGoalStates(start_ompl, goal_ompl);
  problem_setup_->setup();
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
      Eigen::Vector3d(map_pos(0) - roi_ratio * map_width_x, map_pos(1) - roi_ratio * map_width_y, min_elevation)};
  Eigen::Vector3d upper_bounds{
      Eigen::Vector3d(map_pos(0) + roi_ratio * map_width_x, map_pos(1) + roi_ratio * map_width_y, max_elevation)};
  std::cout << "[TerrainOmplRrt] Upper bounds: " << upper_bounds.transpose() << std::endl;
  std::cout << "[TerrainOmplRrt] Lower bounds: " << lower_bounds.transpose() << std::endl;
  setBounds(lower_bounds, upper_bounds);
}

bool TerrainOmplRrt::Solve(double time_budget, TrajectorySegments& path) {
  if (problem_setup_->solve(time_budget)) {
    std::cout << "Found solution:" << std::endl;
    // problem_setup_.getSolutionPath().print(std::cout);
    // problem_setup_.simplifySolution();
    // problem_setup_.getSolutionPath().print(std::cout);
    problem_setup_->getPlannerData(*planner_data_);
    solve_duration_ = problem_setup_->getLastPlanComputationTime();

  } else {
    std::cout << "Solution Not found" << std::endl;
  }

  if (problem_setup_->haveExactSolutionPath()) {
    solutionPathToTrajectorySegments(problem_setup_->getSolutionPath(), path);
    return true;
  }
  return false;
}

bool TerrainOmplRrt::Solve(double time_budget, std::vector<Eigen::Vector3d>& path) {
  if (problem_setup_->solve(time_budget)) {
    std::cout << "Found solution:" << std::endl;
    // problem_setup_.getSolutionPath().print(std::cout);
    // problem_setup_.simplifySolution();
    // problem_setup_.getSolutionPath().print(std::cout);

    problem_setup_->getPlannerData(*planner_data_);
    solve_duration_ = problem_setup_->getLastPlanComputationTime();

  } else {
    std::cout << "Solution Not found" << std::endl;
  }

  if (problem_setup_->haveExactSolutionPath()) {
    solutionPathToTrajectoryPoints(problem_setup_->getSolutionPath(), path);
    return true;
  }
  return false;
}

void TerrainOmplRrt::solutionPathToTrajectorySegments(ompl::geometric::PathGeometric path,
                                                      TrajectorySegments& trajectory_segments) const {
  trajectory_segments.segments.clear();
  path.interpolate();
  std::vector<ompl::base::State*>& state_vector = path.getStates();
  double prev_yaw;
  Trajectory trajectory;
  double prev_curvature = std::numeric_limits<double>::infinity();
  bool is_arc_segment{false};
  double maximum_curvature = 1 / problem_setup_->getGeometricComponentStateSpace()
                                     ->as<fw_planning::spaces::DubinsAirplaneStateSpace>()
                                     ->getMinTurningRadius();
  Eigen::Vector3d prev_position{Eigen::Vector3d::Zero()};  // TODO: Invalidate with nans?
  for (ompl::base::State* state_ptr : state_vector) {
    // Get states from solution path
    Eigen::Vector3d position = dubinsairplanePosition(state_ptr);
    double yaw = dubinsairplaneYaw(state_ptr);

    if (prev_position.norm() < 0.1) {
      // First segment
      prev_position = position;
      continue;
    }

    const Eigen::Vector3d velocity = (position - prev_position).transpose();
    const Eigen::Vector2d velocity_2d = Eigen::Vector2d(velocity(0), velocity(1)).normalized();
    State state;
    state.position = position;
    state.velocity = velocity.normalized();
    state.attitude = Eigen::Vector4d(std::cos(yaw / 2), 0.0, 0.0, std::sin(yaw / 2));
    trajectory.states.emplace_back(state);

    double heading = std::atan2(velocity_2d(1), velocity_2d(0));
    double error = heading - yaw;
    if (error > M_PI)
      error -= M_PI * 2.0;
    else if (error > M_PI)
      error += M_PI * 2.0;

    double curvature{0.0};
    if (std::abs(yaw - heading) > 0.0000001) {  // Arc segments
                                                /// TODO: Get curvature from planner
      curvature = maximum_curvature * (heading - yaw) / std::abs(heading - yaw);
    } else {  // Straight segments
      curvature = 0.0;
    }

    if ((std::abs(prev_curvature - curvature) > 0.0001) && !trajectory.states.empty() &&
        std::isfinite(prev_curvature)) {
      // Segment type has changed
      trajectory_segments.segments.push_back(trajectory);
      trajectory_segments.segments.back().curvature = prev_curvature;
      trajectory.states.clear();
    }

    prev_yaw = yaw;
    prev_curvature = curvature;
    prev_position = position;
    if (state_ptr == state_vector.back()) {
      if (!trajectory.states.empty()) {
        trajectory_segments.segments.push_back(trajectory);
        trajectory_segments.segments.back().curvature = prev_curvature;
        trajectory.states.clear();
      }
    }
  }
}

void TerrainOmplRrt::solutionPathToTrajectoryPoints(ompl::geometric::PathGeometric path,
                                                    std::vector<Eigen::Vector3d>& trajectory_points) const {
  trajectory_points.clear();
  path.interpolate();
  // trajectory_points->reserve(path.getStateCount());

  std::vector<ompl::base::State*>& state_vector = path.getStates();

  for (ompl::base::State* state_ptr : state_vector) {
    auto position = dubinsairplanePosition(state_ptr);
    trajectory_points.emplace_back(position);
  }
}
