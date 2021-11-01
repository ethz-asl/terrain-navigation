/****************************************************************************
 *
 *   Copyright (c) 2021 Jaeyoung Lim. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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
/**
 * @brief View Planner class
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include "terrain_planner/terrain_map.h"
#include "terrain_planner/trajectory.h"

#include <Eigen/Dense>
#include <memory>
#include <vector>

class ManeuverLibrary {
 public:
  ManeuverLibrary();
  virtual ~ManeuverLibrary();
  std::vector<TrajectorySegments>& generateMotionPrimitives(const Eigen::Vector3d current_pos,
                                                            const Eigen::Vector3d current_vel,
                                                            TrajectorySegments& current_path);
  std::vector<TrajectorySegments>& getMotionPrimitives() { return motion_primitives_; }
  std::vector<TrajectorySegments>& getValidPrimitives() { return valid_primitives_; }
  TrajectorySegments& getBestPrimitive();
  TrajectorySegments& getRandomPrimitive();
  Trajectory generateArcTrajectory(Eigen::Vector3d rates, const double horizon, Eigen::Vector3d current_pos,
                                   Eigen::Vector3d current_vel);
  double getPlanningHorizon() { return planning_horizon_; };
  void setPlanningHorizon(double horizon) { planning_horizon_ = horizon; };
  void setTerrainMap(const std::string& map_path) {
    terrain_map_->initializeFromGeotiff(map_path);
    terrain_map_->AddLayerDistanceTransform("distance_surface");
  };
  void setGoalPosition(const Eigen::Vector3d& pos) { goal_pos_ = pos; };
  bool Solve();
  grid_map::GridMap& getGridMap() { return terrain_map_->getGridMap(); };
  double getTimeStep() { return dt_; }

 private:
  static Eigen::Vector4d rpy2quaternion(double roll, double pitch, double yaw);
  std::vector<TrajectorySegments> AppendSegment(std::vector<TrajectorySegments>& first_segment,
                                                const std::vector<Eigen::Vector3d>& rates, const double horizon);
  std::vector<TrajectorySegments> checkCollisions();
  std::vector<TrajectorySegments> checkRelaxedCollisions();
  bool checkTrajectoryCollision(TrajectorySegments& trajectory, const std::string& layer, bool is_above = true);
  double getTrajectoryCollisionCost(TrajectorySegments& trajectory, const std::string& layer, bool is_above = true);

  std::shared_ptr<TerrainMap> terrain_map_;

  // Planner configurations
  std::vector<TrajectorySegments> motion_primitives_;
  std::vector<TrajectorySegments> valid_primitives_;
  std::vector<Eigen::Vector3d> primitive_rates_;
  int num_segments{3};
  Eigen::Vector3d goal_pos_;
  double dt_{0.1};
  double planning_horizon_{10.0};
  double cruise_speed_{15.0};
};
