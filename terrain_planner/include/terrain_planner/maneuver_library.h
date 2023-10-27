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
#ifndef TERRAIN_PLANNER_MANEUVER_LIBRARY_H
#define TERRAIN_PLANNER_MANEUVER_LIBRARY_H

#include "terrain_navigation/primitive.h"
#include "terrain_navigation/terrain_map.h"
#include "terrain_navigation/viewpoint.h"

#include <Eigen/Dense>
#include <memory>
#include <vector>

class ManeuverLibrary {
 public:
  ManeuverLibrary();
  virtual ~ManeuverLibrary();
  std::shared_ptr<Primitive>& generateMotionPrimitives(const Eigen::Vector3d current_pos,
                                                       const Eigen::Vector3d current_vel,
                                                       const Eigen::Vector4d current_att, Path& current_path,
                                                       bool add_emergency = true, int tree_depth = 2,
                                                       double planning_horizon = 10.0);
  std::vector<Path>& getMotionPrimitives() { return motion_primitives_; }
  std::vector<Path>& getValidPrimitives() { return valid_primitives_; }
  Path getRandomPrimitive();
  PathSegment generateArcTrajectory(Eigen::Vector3d rates, const double horizon, Eigen::Vector3d current_pos,
                                    Eigen::Vector3d current_vel, const double dt = 0.1);
  PathSegment generateCircleTrajectory(Eigen::Vector3d center_pos, double radius, const double dt = 0.1);

  /**
   * @brief Get the Primitive Rates of the maneuver library
   *
   * @return std::vector<Eigen::Vector3d>
   */
  std::vector<Eigen::Vector3d> getPrimitiveRates() const { return primitive_rates_; };
  Eigen::Vector3d getRandomPrimitiveRate() const;
  Eigen::Vector3d getGoalPosition() { return goal_pos_; };

  /**
   * @brief Set the Terrain Map object for collision checking
   *
   * @param map terrain map pointer
   */
  void setTerrainMap(std::shared_ptr<TerrainMap> map) { terrain_map_ = map; };

  /**
   * @brief Check if assigned terrain map exists
   *
   * @return true
   * @return false
   */
  bool hasTerrainMap() { return bool(terrain_map_); };
  Eigen::Vector3d setTerrainRelativeGoalPosition(const Eigen::Vector3d& pos);
  void setGoalPosition(const Eigen::Vector3d& pos) { goal_pos_ = pos; };
  bool Solve();
  grid_map::GridMap& getGridMap() { return terrain_map_->getGridMap(); };
  std::shared_ptr<TerrainMap>& getTerrainMap() { return terrain_map_; };
  void expandPrimitives(std::shared_ptr<Primitive> primitive, std::vector<Eigen::Vector3d> rates, double horizon);

  /**
   * @brief
   *
   * @param primitive
   * @return true
   * @return false
   */
  bool updateValidity(std::shared_ptr<Primitive>& primitive);

  /**
   * @brief Check collision of the current segment and child nodes
   *
   * @param primitive
   * @param valid_primitives
   * @param check_valid_child
   * @return true
   * @return false
   */
  bool checkCollisionsTree(std::shared_ptr<Primitive>& primitive, std::vector<Path>& valid_primitives,
                           bool check_valid_child = true);

  static std::vector<ViewPoint> sampleViewPointFromPath(Path& segment);
  static std::vector<ViewPoint> sampleViewPointFromPathSegment(PathSegment& segment);

 private:
  static Eigen::Vector4d rpy2quaternion(double roll, double pitch, double yaw);
  std::vector<Path> AppendSegment(std::vector<Path>& first_segment, const std::vector<Eigen::Vector3d>& rates,
                                  const double horizon);
  bool checkCollisions();
  std::vector<Path> checkRelaxedCollisions();

  bool checkTrajectoryCollision(PathSegment& trajectory, const std::string& layer, bool is_above = true);
  bool checkTrajectoryCollision(Path& trajectory, const std::string& layer, bool is_above = true);
  double getTrajectoryCollisionCost(Path& trajectory, const std::string& layer, bool is_above = true);

  std::shared_ptr<TerrainMap> terrain_map_;

  // Planner configurations
  std::vector<Path> motion_primitives_;
  std::vector<Path> valid_primitives_;
  std::vector<Eigen::Vector3d> primitive_rates_;
  std::shared_ptr<Primitive> motion_primitive_tree_;
  int num_segments{3};
  Eigen::Vector3d goal_pos_{Eigen::Vector3d(0.0, 0.0, 100.0)};  // Terrain relative goal position
  double cruise_speed_{20.0};
  double goal_terrain_altitude_{100.0};
};
#endif
