//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef TERRAIN_PLANNER_TERRAIN_OMPL_RRT_H
#define TERRAIN_PLANNER_TERRAIN_OMPL_RRT_H

#include <stdio.h>
#include <cstdlib>
#include <sstream>
#include <string>

#include <Eigen/Dense>

#include <terrain_navigation/terrain_map.h>
#include "terrain_navigation/trajectory.h"

#include "terrain_planner/ompl_setup.h"

#include <ompl/base/goals/GoalStates.h>

class TerrainOmplRrt {
 public:
  TerrainOmplRrt();
  virtual ~TerrainOmplRrt();

  void setupProblem(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& goal);
  void setupProblem(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel, const Eigen::Vector3d& goal,
                    const Eigen::Vector3d& goal_vel);
  void setBounds(const Eigen::Vector3d& lower_bound, const Eigen::Vector3d& upper_bound) {
    lower_bound_ = lower_bound;
    upper_bound_ = upper_bound;
  }
  void setBoundsFromMap(const grid_map::GridMap& map);
  void setMap(std::shared_ptr<TerrainMap> map) { map_ = std::move(map); }
  bool Solve(double time_budget, TrajectorySegments& path);
  bool Solve(double time_budget, std::vector<Eigen::Vector3d>& path);
  double getSegmentCurvature(std::shared_ptr<ompl::OmplSetup> problem_setup,
                             fw_planning::spaces::DubinsPath& dubins_path, const size_t start_idx) const;
  void solutionPathToTrajectorySegments(ompl::geometric::PathGeometric path, TrajectorySegments& trajectory_segments,
                                        double resolution = 0.05) const;
  void solutionPathToTrajectoryPoints(ompl::geometric::PathGeometric path,
                                      std::vector<Eigen::Vector3d>& trajectory_points) const;
  std::shared_ptr<ompl::base::PlannerData> getPlannerData() { return planner_data_; };
  std::shared_ptr<ompl::OmplSetup> getProblemSetup() { return problem_setup_; };
  ompl::base::StateSamplerPtr allocTerrainStateSampler(const ompl::base::StateSpace* space) {
    return std::make_shared<ompl::TerrainStateSampler>(space, map_->getGridMap(), max_altitude_, min_altitude_);
  }
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
  std::shared_ptr<ompl::OmplSetup> problem_setup_;
  std::shared_ptr<TerrainMap> map_;
  double min_altitude_{50.0};
  double max_altitude_{150.0};
  std::shared_ptr<ompl::base::PlannerData> planner_data_;
  std::shared_ptr<ompl::base::GoalStates> goal_states_;
  Eigen::Vector3d lower_bound_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d upper_bound_{Eigen::Vector3d::Zero()};
  double solve_duration_{0.0};
};

#endif
