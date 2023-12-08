/****************************************************************************
 *
 *   Copyright (c) 2021 Jaeyoung Lim, Autonomous Systems Lab,
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

#ifndef TERRAIN_PLANNER_TERRAIN_OMPL_RRT_H
#define TERRAIN_PLANNER_TERRAIN_OMPL_RRT_H

#include <ompl/base/goals/GoalStates.h>
#include <stdio.h>
#include <terrain_navigation/terrain_map.h>

#include <Eigen/Dense>
#include <cstdlib>
#include <sstream>
#include <string>

#include "terrain_navigation/path.h"
#include "terrain_planner/ompl_setup.h"

class TerrainOmplRrt {
 public:
  TerrainOmplRrt();
  TerrainOmplRrt(const ompl::base::StateSpacePtr& space);
  virtual ~TerrainOmplRrt();

  /**
   * @brief Configure OMPL problem descriptions
   *
   */
  void configureProblem();

  /**
   * @brief Setup problem with center position of start and goal loiter circles
   *
   * @param start_pos center of the start loiter position
   * @param goal center of the goal loiter position
   * @param start_loiter_radius Specify direction of the start circle.
   *          - Positive: anti clockwise
   *          - Negative: Clockwise
   */
  void setupProblem(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& goal) {
    this->setupProblem(
        start_pos, goal,
        problem_setup_->getStateSpace()->as<fw_planning::spaces::DubinsAirplaneStateSpace>()->getMinTurningRadius());
  };

  /**
   * @brief Setup problem with center position of start and goal loiter circle with specific radius
   *
   * @param start_pos
   * @param goal
   * @param start_loiter_radius
   */
  void setupProblem(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& goal, double start_loiter_radius);

  /**
   * @brief Setup problem with position, velocity of the start and center of the goal loiter circle
   *
   * @param start_pos position of the start state
   * @param start_vel velocity of the start state
   * @param goal center of the goal loiter position
   */
  void setupProblem(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel, const Eigen::Vector3d& goal,
                    double goal_radius = -1);

  /**
   * @brief Setup problem with position, velocity of the start and goal state
   *
   * @param start_pos position of the start state
   * @param start_vel velocity of the start state
   * @param goal position of the goal state
   * @param goal_vel velocity of the goal state
   */
  void setupProblem(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel,
                    const std::vector<Eigen::Vector3d>& goal_positions);

  /**
   * @brief Setup problem with position, velocity of the start and goal state
   *
   * @param start_pos position of the start state
   * @param start_vel velocity of the start state
   * @param goal position of the goal state
   * @param goal_vel velocity of the goal state
   */
  void setupProblem(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel, const Eigen::Vector3d& goal,
                    const Eigen::Vector3d& goal_vel);

  /**
   * @brief Set the Bounds of the statespace for the planner
   *
   * @param lower_bound lower bo
   * @param upper_bound
   */
  void setBounds(const Eigen::Vector3d& lower_bound, const Eigen::Vector3d& upper_bound) {
    lower_bound_ = lower_bound;
    upper_bound_ = upper_bound;
  }

  /**
   * @brief Set the Bounds of the statespace for the planner using the map
   *
   * @param map
   */
  void setBoundsFromMap(const grid_map::GridMap& map);

  /**
   * @brief Set the Map
   *
   * @param map
   */
  void setMap(std::shared_ptr<TerrainMap> map) { map_ = std::move(map); }

  /**
   * @brief Set the Max Altitude Collision Check
   *
   * @param check_max_altitude If true, enables the maximum altitude collision checks
   */
  void setMaxAltitudeCollisionChecks(bool check_max_altitude) { check_max_altitude_ = check_max_altitude; }

  /**
   * @brief Solve the planning problem for a given time budget, and return a TerrainSegments object if an exact solution
   * is found
   *
   * @param time_budget [s] time the planner should use for planning
   * @param path
   * @return true Found exact solution
   * @return false Did not find an exact solution
   */
  bool Solve(double time_budget, Path& path);
  bool Solve(double time_budget, std::vector<Eigen::Vector3d>& path);
  double getSegmentCurvature(std::shared_ptr<ompl::OmplSetup> problem_setup,
                             fw_planning::spaces::DubinsPath& dubins_path, const size_t start_idx) const;
  void solutionPathToPath(ompl::geometric::PathGeometric path, Path& trajectory_segments,
                          double resolution = 0.05) const;
  void solutionPathToTrajectoryPoints(ompl::geometric::PathGeometric path,
                                      std::vector<Eigen::Vector3d>& trajectory_points) const;
  std::shared_ptr<ompl::base::PlannerData> getPlannerData() { return planner_data_; };
  std::shared_ptr<ompl::OmplSetup> getProblemSetup() { return problem_setup_; };
  ompl::base::StateSamplerPtr allocTerrainStateSampler(const ompl::base::StateSpace* space) {
    return std::make_shared<ompl::TerrainStateSampler>(space, map_->getGridMap(), max_altitude_, min_altitude_);
  }
  bool getSolutionPathLength(double& path_length);
  bool getSolutionPath(std::vector<Eigen::Vector3d>& path);
  double getSolutionTime() { return solve_duration_; };
  static Eigen::Vector3d dubinsairplanePosition(ompl::base::State* state_ptr) {
    Eigen::Vector3d position(state_ptr->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getX(),
                             state_ptr->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getY(),
                             state_ptr->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getZ());
    return position;
  }
  static double dubinsairplaneYaw(ompl::base::State* state_ptr) {
    double yaw = state_ptr->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getYaw();
    return yaw;
  }
  static inline void segmentStart2omplState(fw_planning::spaces::DubinsAirplaneStateSpace::SegmentStarts::Start start,
                                            ompl::base::State* state) {
    state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setX(start.x);
    state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setY(start.y);
    state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setZ(start.z);
    state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setYaw(start.yaw);
  }
  void setAltitudeLimits(const double max_altitude, const double min_altitude) {
    max_altitude_ = max_altitude;
    min_altitude_ = min_altitude;
  }

 private:
  // double minimum_turning_radius_{66.67};
  std::shared_ptr<ompl::OmplSetup> problem_setup_;
  std::shared_ptr<TerrainMap> map_;
  double min_altitude_{50.0};
  double max_altitude_{120.0};
  std::shared_ptr<ompl::base::PlannerData> planner_data_;
  std::shared_ptr<ompl::base::GoalStates> goal_states_;
  Eigen::Vector3d lower_bound_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d upper_bound_{Eigen::Vector3d::Zero()};
  double solve_duration_{0.0};
  bool check_max_altitude_{true};
};

#endif
