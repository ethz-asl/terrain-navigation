/****************************************************************************
 *
 *   Copyright (c) 2024 Jaeyoung Lim, Autonomous Systems Lab,
 *  ETH Zürich. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "terrain_planner/terrain_mm_prm.h"
#include <ompl/geometric/planners/prm/PRM.h>

// Constructor
TerrainMmPrm::TerrainMmPrm() {
  problem_setup_ =
      std::make_shared<ompl::OmplSetup>(ompl::base::StateSpacePtr(new fw_planning::spaces::DubinsAirplaneStateSpace()));
}
TerrainMmPrm::TerrainMmPrm(const ompl::base::StateSpacePtr& space) {
  problem_setup_ = std::make_shared<ompl::OmplSetup>(space);
}
TerrainMmPrm::~TerrainMmPrm() {
  // Destructor
}

void TerrainMmPrm::configureProblem() {
  problem_setup_->clear();
  problem_setup_->clearStartStates();

  auto planner = std::make_shared<ompl::geometric::PRM>(problem_setup_->getSpaceInformation());
  problem_setup_->setPlanner(planner);

  // shortest path objective
  problem_setup_->setDefaultObjective();
  // assert(map);
  problem_setup_->setTerrainCollisionChecking(map_->getGridMap(), check_max_altitude_);
  // problem_setup_->getStateSpace()->setStateSamplerAllocator(
  //     std::bind(&TerrainMmPrm::allocTerrainStateSampler, this, std::placeholders::_1));
  // problem_setup_->getStateSpace()->allocStateSampler();
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
}

void TerrainMmPrm::setupProblem(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& goal,
                                double start_loiter_radius) {
  configureProblem();
  double radius =
      problem_setup_->getStateSpace()->as<fw_planning::spaces::DubinsAirplaneStateSpace>()->getMinTurningRadius();
  double delta_theta = 0.1;
  for (double theta = -M_PI; theta < M_PI; theta += (delta_theta * 2 * M_PI)) {
    ompl::base::ScopedState<fw_planning::spaces::DubinsAirplaneStateSpace> start_ompl(
        problem_setup_->getSpaceInformation());

    start_ompl->setX(start_pos(0) + std::abs(start_loiter_radius) * std::cos(theta));
    start_ompl->setY(start_pos(1) + std::abs(start_loiter_radius) * std::sin(theta));
    start_ompl->setZ(start_pos(2));
    double start_yaw = bool(start_loiter_radius > 0) ? theta - M_PI_2 : theta + M_PI_2;
    wrap_pi(start_yaw);
    start_ompl->setYaw(start_yaw);
    problem_setup_->addStartState(start_ompl);
  }

  goal_states_ = std::make_shared<ompl::base::GoalStates>(problem_setup_->getSpaceInformation());
  for (double theta = -M_PI; theta < M_PI; theta += (delta_theta * 2 * M_PI)) {
    ompl::base::ScopedState<fw_planning::spaces::DubinsAirplaneStateSpace> goal_ompl(
        problem_setup_->getSpaceInformation());
    goal_ompl->setX(goal(0) + radius * std::cos(theta));
    goal_ompl->setY(goal(1) + radius * std::sin(theta));
    goal_ompl->setZ(goal(2));
    double goal_yaw = theta + M_PI_2;
    wrap_pi(goal_yaw);
    goal_ompl->setYaw(goal_yaw);
    goal_states_->addState(goal_ompl);
    goal_yaw = theta - M_PI_2;
    wrap_pi(goal_yaw);
    goal_ompl->setYaw(goal_yaw);
    goal_states_->addState(goal_ompl);  // Add additional state for bidirectional tangents
  }
  problem_setup_->setGoal(goal_states_);

  problem_setup_->setup();

  auto planner_ptr = problem_setup_->getPlanner();
  // std::cout << "Planner Range: " << planner_ptr->as<ompl::geometric::RRTstar>()->getRange() << std::endl;
}

void TerrainMmPrm::setupProblem(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel,
                                const Eigen::Vector3d& goal, double goal_radius) {
  configureProblem();

  double radius;
  if (goal_radius < 0) {
    radius =
        problem_setup_->getStateSpace()->as<fw_planning::spaces::DubinsAirplaneStateSpace>()->getMinTurningRadius();
  } else {
    radius = goal_radius;
  }
  double delta_theta = 0.1;
  ompl::base::ScopedState<fw_planning::spaces::DubinsAirplaneStateSpace> start_ompl(
      problem_setup_->getSpaceInformation());

  start_ompl->setX(start_pos(0));
  start_ompl->setY(start_pos(1));
  start_ompl->setZ(start_pos(2));
  double start_yaw = std::atan2(start_vel(1), start_vel(0));
  start_ompl->setYaw(start_yaw);
  problem_setup_->clearStartStates();  // Clear previous goal states
  problem_setup_->addStartState(start_ompl);

  goal_states_ = std::make_shared<ompl::base::GoalStates>(problem_setup_->getSpaceInformation());
  for (double theta = -M_PI; theta < M_PI; theta += (delta_theta * 2 * M_PI)) {
    ompl::base::ScopedState<fw_planning::spaces::DubinsAirplaneStateSpace> goal_ompl(
        problem_setup_->getSpaceInformation());
    goal_ompl->setX(goal(0) + radius * std::cos(theta));
    goal_ompl->setY(goal(1) + radius * std::sin(theta));
    goal_ompl->setZ(goal(2));
    double goal_yaw = theta + M_PI_2;
    wrap_pi(goal_yaw);
    goal_ompl->setYaw(goal_yaw);
    goal_states_->addState(goal_ompl);
    goal_yaw = theta - M_PI_2;
    wrap_pi(goal_yaw);
    goal_ompl->setYaw(goal_yaw);
    goal_states_->addState(goal_ompl);  // Add additional state for bidirectional tangents
  }
  problem_setup_->setGoal(goal_states_);

  problem_setup_->setup();
}

void TerrainMmPrm::setupProblem(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel,
                                const std::vector<Eigen::Vector3d>& goal_positions) {
  if (goal_positions.empty()) {
    std::cout << "Failed to configure problem: Goal position list empty" << std::endl;
    return;
  }
  configureProblem();

  double radius =
      problem_setup_->getStateSpace()->as<fw_planning::spaces::DubinsAirplaneStateSpace>()->getMinTurningRadius();
  double delta_theta = 0.1;
  ompl::base::ScopedState<fw_planning::spaces::DubinsAirplaneStateSpace> start_ompl(
      problem_setup_->getSpaceInformation());

  start_ompl->setX(start_pos(0));
  start_ompl->setY(start_pos(1));
  start_ompl->setZ(start_pos(2));
  double start_yaw = std::atan2(start_vel(1), start_vel(0));
  start_ompl->setYaw(start_yaw);
  problem_setup_->clearStartStates();  // Clear previous goal states
  problem_setup_->addStartState(start_ompl);

  goal_states_ = std::make_shared<ompl::base::GoalStates>(problem_setup_->getSpaceInformation());
  for (auto& goal : goal_positions) {
    for (double theta = -M_PI; theta < M_PI; theta += (delta_theta * 2 * M_PI)) {
      ompl::base::ScopedState<fw_planning::spaces::DubinsAirplaneStateSpace> goal_ompl(
          problem_setup_->getSpaceInformation());
      goal_ompl->setX(goal(0) + radius * std::cos(theta));
      goal_ompl->setY(goal(1) + radius * std::sin(theta));
      goal_ompl->setZ(goal(2));
      double goal_yaw = theta + M_PI_2;
      wrap_pi(goal_yaw);
      goal_ompl->setYaw(goal_yaw);
      goal_states_->addState(goal_ompl);
      goal_yaw = theta - M_PI_2;
      wrap_pi(goal_yaw);
      goal_ompl->setYaw(goal_yaw);
      goal_states_->addState(goal_ompl);  // Add additional state for bidirectional tangents
    }
  }
  problem_setup_->setGoal(goal_states_);

  problem_setup_->setup();
}

void TerrainMmPrm::setupProblem(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel,
                                const Eigen::Vector3d& goal, const Eigen::Vector3d& goal_vel) {
  configureProblem();

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

void TerrainMmPrm::setBoundsFromMap(const grid_map::GridMap& map) {
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
  std::cout << "[TerrainMmPrm] Upper bounds: " << upper_bounds.transpose() << std::endl;
  std::cout << "[TerrainMmPrm] Lower bounds: " << lower_bounds.transpose() << std::endl;
  setBounds(lower_bounds, upper_bounds);
}

bool TerrainMmPrm::Solve(double time_budget, Path& path) {
  if (problem_setup_->solve(time_budget)) {
    // problem_setup_.getSolutionPath().print(std::cout);
    // problem_setup_.simplifySolution();
    // problem_setup_.getSolutionPath().print(std::cout);
    problem_setup_->getPlannerData(*planner_data_);
    solve_duration_ = problem_setup_->getLastPlanComputationTime();

  } else {
    std::cout << "Solution Not found" << std::endl;
  }

  if (problem_setup_->haveExactSolutionPath()) {
    std::cout << "Found Exact solution!" << std::endl;
    solutionPathToPath(problem_setup_->getSolutionPath(), path);
    return true;
  }
  return false;
}

bool TerrainMmPrm::Solve(double time_budget, std::vector<Eigen::Vector3d>& path) {
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

bool TerrainMmPrm::getSolutionPathLength(double& path_length) {
  if (problem_setup_->haveExactSolutionPath()) {
    ompl::geometric::PathGeometric path = problem_setup_->getSolutionPath();
    path.interpolate();
    path_length = path.length();
    return true;
  }
  return false;
}

bool TerrainMmPrm::getSolutionPath(std::vector<Eigen::Vector3d>& path) {
  if (problem_setup_->haveExactSolutionPath()) {
    solutionPathToTrajectoryPoints(problem_setup_->getSolutionPath(), path);
    return true;
  }
  return false;
}

double TerrainMmPrm::getSegmentCurvature(std::shared_ptr<ompl::OmplSetup> problem_setup,
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

void TerrainMmPrm::solutionPathToPath(ompl::geometric::PathGeometric path, Path& trajectory_segments,
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

    const double total_length = dubins_path.length_2D();
    const double dt = resolution / dubins_path.length_2D();
    double progress{0.0};
    for (size_t start_idx = 0; start_idx < segmentStarts.segmentStarts.size(); start_idx++) {
      if (dubins_path.getSegmentLength(start_idx) > 0.0) {
        double segment_progress = dubins_path.getSegmentLength(start_idx) / total_length;
        // Read segment start and end statess
        segmentStart2omplState(segmentStarts.segmentStarts[start_idx], segment_start_state);
        if ((start_idx + 1) > (segmentStarts.segmentStarts.size() - 1)) {
          segment_end_state = to;
        } else if ((start_idx + 1) > (segmentStarts.segmentStarts.size() - 2) &&
                   dubins_path.getSegmentLength(start_idx + 1) == 0.0) {
          segment_end_state = to;
        } else {
          segmentStart2omplState(segmentStarts.segmentStarts[start_idx + 1], segment_end_state);
        }

        // Append to trajectory
        PathSegment trajectory;
        trajectory.curvature = getSegmentCurvature(problem_setup_, dubins_path, start_idx);
        ompl::base::State* state = problem_setup_->getStateSpace()->allocState();
        trajectory.flightpath_angle = dubins_path.getGamma();
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
        // Append end state
        if (((start_idx + 1) > (segmentStarts.segmentStarts.size() - 1)) ||
            ((start_idx + 1) > (segmentStarts.segmentStarts.size() - 2) &&
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
      }
    }
  }
}

void TerrainMmPrm::solutionPathToTrajectoryPoints(ompl::geometric::PathGeometric path,
                                                  std::vector<Eigen::Vector3d>& trajectory_points) const {
  trajectory_points.clear();
  path.interpolate();

  std::vector<ompl::base::State*>& state_vector = path.getStates();

  for (ompl::base::State* state_ptr : state_vector) {
    auto position = dubinsairplanePosition(state_ptr);
    trajectory_points.emplace_back(position);
  }
}
