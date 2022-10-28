//  June/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "terrain_planner/terrain_ompl_rrt.h"

// Constructor
TerrainOmplRrt::TerrainOmplRrt() { problem_setup_ = std::make_shared<ompl::OmplSetup>(); }
TerrainOmplRrt::~TerrainOmplRrt() {
  // Destructor
}

void TerrainOmplRrt::setupProblem(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel,
                                  const Eigen::Vector3d& goal, const Eigen::Vector3d& goal_vel) {
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
  double goal_yaw = std::atan2(goal_vel(1), goal_vel(0));
  goal_ompl->setYaw(goal_yaw);

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

bool TerrainOmplRrt::getSolutionPath(std::vector<Eigen::Vector3d>& path) {
  if (problem_setup_->haveExactSolutionPath()) {
    solutionPathToTrajectoryPoints(problem_setup_->getSolutionPath(), path);
    return true;
  }
  return false;
}

double TerrainOmplRrt::getSegmentCurvature(std::shared_ptr<ompl::OmplSetup> problem_setup,
                                           fw_planning::spaces::DubinsPath& dubins_path, const size_t start_idx) const {
  double segment_curvature{0.0};
  double maximum_curvature = 1 / problem_setup->getGeometricComponentStateSpace()
                                     ->as<fw_planning::spaces::DubinsAirplaneStateSpace>()
                                     ->getMinTurningRadius();
  switch (
      dubins_path
          .getType()[problem_setup->getStateSpace()->as<fw_planning::spaces::DubinsAirplaneStateSpace>()->convert_idx(
              start_idx)]) {
    case fw_planning::spaces::DubinsPath::DUBINS_LEFT:
      segment_curvature = maximum_curvature;
      break;
    case fw_planning::spaces::DubinsPath::DUBINS_RIGHT:
      segment_curvature = -maximum_curvature;
      break;
    case fw_planning::spaces::DubinsPath::DUBINS_STRAIGHT:
      segment_curvature = 0.0;
      break;
  }
  return segment_curvature;
}

void TerrainOmplRrt::solutionPathToTrajectorySegments(ompl::geometric::PathGeometric path,
                                                      TrajectorySegments& trajectory_segments,
                                                      double resolution) const {
  trajectory_segments.segments.clear();

  std::vector<ompl::base::State*>& state_vector = path.getStates();
  for (size_t idx = 0; idx < state_vector.size() - 1; idx++) {
    auto from = state_vector[idx];    // Start of the segment
    auto to = state_vector[idx + 1];  // End of the segment
    fw_planning::spaces::DubinsPath dubins_path;
    problem_setup_->getStateSpace()->as<fw_planning::spaces::DubinsAirplaneStateSpace>()->dubins(from, to, dubins_path);
    fw_planning::spaces::DubinsAirplaneStateSpace::SegmentStarts segmentStarts;
    problem_setup_->getStateSpace()->as<fw_planning::spaces::DubinsAirplaneStateSpace>()->calculateSegments(
        from, to, dubins_path, segmentStarts);

    ompl::base::State* segment_start_state = problem_setup_->getStateSpace()->allocState();
    ompl::base::State* segment_end_state = problem_setup_->getStateSpace()->allocState();

    const double total_length = dubins_path.length_3D();
    const double dt = resolution / dubins_path.length_3D();
    double progress{0.0};
    for (size_t start_idx = 0; start_idx < segmentStarts.segmentStarts.size() - 1; start_idx++) {
      if (dubins_path.getSegmentLength(start_idx) > 0.0) {
        double segment_progress = dubins_path.getSegmentLength(start_idx) / total_length;
        // Read segment start and end statess
        segmentStart2omplState(segmentStarts.segmentStarts[start_idx], segment_start_state);
        if ((start_idx + 1) == (segmentStarts.segmentStarts.size() - 1)) {
          segment_end_state = to;
        } else if ((start_idx + 1) == (segmentStarts.segmentStarts.size() - 2) &&
                   dubins_path.getSegmentLength(start_idx + 1) == 0.0) {
          segment_end_state = to;
        } else {
          segmentStart2omplState(segmentStarts.segmentStarts[start_idx + 1], segment_end_state);
        }

        // Append to trajectory
        Trajectory trajectory;
        trajectory.curvature = getSegmentCurvature(problem_setup_, dubins_path, start_idx);
        ompl::base::State* state = problem_setup_->getStateSpace()->allocState();
        double yaw;
        double track_progress{0.0};
        for (double t = progress; t <= progress + segment_progress; t = t + dt) {
          State segment_state;
          problem_setup_->getStateSpace()->as<fw_planning::spaces::DubinsAirplaneStateSpace>()->interpolate(
              dubins_path, segmentStarts, t, state);
          Eigen::Vector3d position = dubinsairplanePosition(state);
          yaw = dubinsairplaneYaw(state);
          Eigen::Vector3d velocity = Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0.0);
          segment_state.position = position;
          segment_state.velocity = velocity;
          segment_state.attitude = Eigen::Vector4d(std::cos(yaw / 2.0), 0.0, 0.0, std::sin(yaw / 2.0));
          trajectory.states.emplace_back(segment_state);
          track_progress = t;
        }
        if (((start_idx + 1) == (segmentStarts.segmentStarts.size() - 1)) ||
            ((start_idx + 1) == (segmentStarts.segmentStarts.size() - 2) &&
             dubins_path.getSegmentLength(start_idx + 1) == 0.0)) {
          // Append segment with last state
          State end_state;
          Eigen::Vector3d end_position = dubinsairplanePosition(segment_end_state);
          double end_yaw = dubinsairplaneYaw(segment_end_state);
          Eigen::Vector3d end_velocity = Eigen::Vector3d(std::cos(end_yaw), std::sin(end_yaw), 0.0);
          end_state.position = end_position;
          end_state.velocity = end_velocity;
          end_state.attitude = Eigen::Vector4d(std::cos(end_yaw / 2.0), 0.0, 0.0, std::sin(end_yaw / 2.0));
          trajectory.states.emplace_back(end_state);
        }
        progress = track_progress;
        // Do not append trajectory if the segment is too short
        if (trajectory.states.size() > 1) {
          trajectory_segments.segments.push_back(trajectory);
        }
      } else {
        segmentStart2omplState(segmentStarts.segmentStarts[start_idx], segment_start_state);
        if ((start_idx + 1) == (segmentStarts.segmentStarts.size() - 1)) {
          segment_end_state = to;
        } else {
          segmentStart2omplState(segmentStarts.segmentStarts[start_idx + 1], segment_end_state);
        }
      }
    }
  }
}

void TerrainOmplRrt::solutionPathToTrajectoryPoints(ompl::geometric::PathGeometric path,
                                                    std::vector<Eigen::Vector3d>& trajectory_points) const {
  trajectory_points.clear();
  path.interpolate();

  std::vector<ompl::base::State*>& state_vector = path.getStates();

  for (ompl::base::State* state_ptr : state_vector) {
    auto position = dubinsairplanePosition(state_ptr);
    trajectory_points.emplace_back(position);
  }
}
