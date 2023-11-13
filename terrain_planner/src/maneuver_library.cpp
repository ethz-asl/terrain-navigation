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
/**
 * @brief View Planner class
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#ifndef MANEUVER_LIBRARY_H
#define MANEUVER_LIBRARY_H

#include "terrain_planner/maneuver_library.h"
#include <iostream>

ManeuverLibrary::ManeuverLibrary() {
  primitive_rates_.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, 3.0, 0.0));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, -3.0, 0.0));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, 0.0, 0.3));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, 3.0, 0.3));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, -3.0, 0.3));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, 0.0, -0.3));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, 3.0, -0.3));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, -3.0, -0.3));
}

ManeuverLibrary::~ManeuverLibrary() {}

std::shared_ptr<Primitive> &ManeuverLibrary::generateMotionPrimitives(const Eigen::Vector3d current_pos,
                                                                      const Eigen::Vector3d current_vel,
                                                                      const Eigen::Vector4d current_att,
                                                                      Path &current_path, bool add_emergency,
                                                                      int tree_depth, double planning_horizon) {
  PathSegment current_segment;
  if (!current_path.segments.empty()) {
    current_segment = current_path.getCurrentSegment(current_pos);
  } else {
    State state_vector;
    state_vector.position = current_pos;
    state_vector.velocity = current_vel.normalized();
    state_vector.attitude = current_att;
    current_segment.states.push_back(state_vector);
  }

  motion_primitive_tree_ = std::make_shared<Primitive>(current_segment);

  // Expand motion primitives
  for (int i = 0; i < tree_depth; i++) {
    expandPrimitives(motion_primitive_tree_, primitive_rates_, planning_horizon);
  }
  if (add_emergency) {  // Add emergency primitives at the end of the trajectory
    std::vector<Eigen::Vector3d> emergency_rates;
    emergency_rates.push_back(Eigen::Vector3d(0.0, 0.0, 0.3));
    double horizon = 2 * M_PI / emergency_rates[0](2);
    expandPrimitives(motion_primitive_tree_, emergency_rates, horizon);
  }

  return motion_primitive_tree_;
}

bool ManeuverLibrary::Solve() {
  checkCollisions();

  if (valid_primitives_.size() < 1) {
    // Try to see if relaxing max altitude fixes the problem
    valid_primitives_ = checkRelaxedCollisions();
    if (valid_primitives_.size() < 1) {
      return false;  // No valid motion primitive
    }
  }
  return true;
}

void ManeuverLibrary::expandPrimitives(std::shared_ptr<Primitive> primitive, std::vector<Eigen::Vector3d> rates,
                                       double horizon) {
  if (primitive->child_primitives.empty()) {
    Eigen::Vector3d current_pos = primitive->getEndofSegmentPosition();
    Eigen::Vector3d current_vel = primitive->getEndofSegmentVelocity();
    for (auto rate : rates) {
      PathSegment trajectory = generateArcTrajectory(rate, horizon, current_pos, current_vel);
      primitive->child_primitives.push_back(std::make_shared<Primitive>(trajectory));
      primitive->child_primitives.back()->depth = primitive->depth + 1;
    }
  } else {
    for (auto &child : primitive->child_primitives) {
      expandPrimitives(child, rates, horizon);
    }
  }
}

bool ManeuverLibrary::updateValidity(std::shared_ptr<Primitive> &primitive) {
  bool segment_valid = primitive->validity;

  // Check recursively if there the leaf has a valid child
  // If there are no valid child available, there the leaf is invalidated
  if (primitive->has_child()) {
    // If the primitive segment is valid and has a child it needs to have at least one child that is valid
    bool has_valid_child{false};
    for (auto &child : primitive->child_primitives) {
      if (updateValidity(child)) {
        has_valid_child = true;
      }
    }
    primitive->validity = has_valid_child && segment_valid;
  }
  return primitive->validity;
}

bool ManeuverLibrary::checkCollisionsTree(std::shared_ptr<Primitive> &primitive, std::vector<Path> &valid_primitives,
                                          bool check_valid_child) {
  bool valid_segment{true};
  // Check collision with terrain
  PathSegment &current_trajectory = primitive->segment;
  valid_segment = valid_segment && checkTrajectoryCollision(current_trajectory, "distance_surface", true);
  // Check collision with maximum terrain altitude
  valid_segment = valid_segment && checkTrajectoryCollision(current_trajectory, "max_elevation", false);
  primitive->validity = valid_segment;

  // Check recursively if there the leaf has a valid child
  // If there are no valid child available, there the leaf is invalidated
  if (primitive->has_child()) {
    // If the primitive segment is valid and has a child it needs to have at least one child that is valid
    bool has_valid_child{false};
    for (auto &child : primitive->child_primitives) {
      std::vector<Path> child_segments;
      if (checkCollisionsTree(child, child_segments)) {
        has_valid_child = true;
        // for (auto &child : child_segments) {
        //   Path valid_segments;
        //   child.prependSegment(current_trajectory);
        //   valid_primitives.push_back(valid_segments);
        // }
      }
    }
    primitive->validity = has_valid_child && valid_segment;
  }
  return primitive->validity;
}

bool ManeuverLibrary::checkCollisions() {
  std::vector<Path> valid_primitives;
  bool valid = checkCollisionsTree(motion_primitive_tree_, valid_primitives);
  valid_primitives_ = valid_primitives;
  return valid;
}

std::vector<Path> ManeuverLibrary::checkRelaxedCollisions() {
  // Iterate through all motion primtiives and only return collision free primitives
  // This is assuming that there are no collision free primitives, therefore the violation of the constraint is measured

  std::vector<Path> valid_primitives;
  for (auto &trajectory : motion_primitives_) {
    double distance_surface_violation = getTrajectoryCollisionCost(trajectory, "distance_surface");
    double max_elevation_violation = getTrajectoryCollisionCost(trajectory, "max_elevation", false);
    trajectory.validity = !(max_elevation_violation > 0.0 || distance_surface_violation > 0.0);
    trajectory.utility = (-1.0) * (max_elevation_violation + distance_surface_violation) / 1000.0;
    valid_primitives.push_back(trajectory);
  }
  return valid_primitives;
}

bool ManeuverLibrary::checkTrajectoryCollision(PathSegment &trajectory, const std::string &layer, bool is_above) {
  /// TODO: Reference gridmap terrain
  for (auto position : trajectory.position()) {
    // TODO: Make max terrain optional
    if (terrain_map_->isInCollision(layer, position, is_above)) {
      return false;
    }
  }
  return true;
}

bool ManeuverLibrary::checkTrajectoryCollision(Path &trajectory, const std::string &layer, bool is_above) {
  /// TODO: Reference gridmap terrain
  for (auto position : trajectory.position()) {
    // TODO: Make max terrain optional
    if (terrain_map_->isInCollision(layer, position, is_above)) {
      return false;
    }
  }
  return true;
}

double ManeuverLibrary::getTrajectoryCollisionCost(Path &trajectory, const std::string &layer, bool is_above) {
  double cost{0.0};
  /// Iterate through whole trajectory to calculate collision depth
  for (auto position : trajectory.position()) {
    cost += terrain_map_->getCollisionDepth(layer, position, is_above);
  }
  return cost;
}

std::vector<Path> ManeuverLibrary::AppendSegment(std::vector<Path> &first_segment,
                                                 const std::vector<Eigen::Vector3d> &rates, const double horizon) {
  // Append second segment for each primitive
  std::vector<Path> second_segment;
  for (auto trajectory_segments : first_segment) {
    for (auto rate : rates) {
      Eigen::Vector3d end_pos = trajectory_segments.lastSegment().states.back().position;
      Eigen::Vector3d end_vel = trajectory_segments.lastSegment().states.back().velocity;
      PathSegment new_segment = generateArcTrajectory(rate, horizon, end_pos, end_vel);
      Path trajectory_2 = trajectory_segments;
      trajectory_2.appendSegment(new_segment);
      second_segment.push_back(trajectory_2);
    }
  }
  return second_segment;
}

/// TODO: Change rate vector for curvature and climbrates
PathSegment ManeuverLibrary::generateArcTrajectory(Eigen::Vector3d rate, const double horizon,
                                                   Eigen::Vector3d current_pos, Eigen::Vector3d current_vel,
                                                   const double dt) {
  PathSegment trajectory;
  trajectory.states.clear();

  double time = 0.0;
  const double current_yaw = std::atan2(-1.0 * current_vel(1), current_vel(0));
  const double climb_rate = rate(1);
  trajectory.flightpath_angle = std::asin(climb_rate / cruise_speed_);
  /// TODO: Fix sign conventions for curvature
  trajectory.curvature = -rate(2) / cruise_speed_;
  trajectory.dt = dt;
  for (int i = 0; i < std::max(1.0, horizon / dt); i++) {
    if (std::abs(rate(2)) < 0.0001) {
      rate(2) > 0.0 ? rate(2) = 0.0001 : rate(2) = -0.0001;
    }
    double yaw = rate(2) * time + current_yaw;

    Eigen::Vector3d pos =
        cruise_speed_ / rate(2) *
            Eigen::Vector3d(std::sin(yaw) - std::sin(current_yaw), std::cos(yaw) - std::cos(current_yaw), 0) +
        Eigen::Vector3d(0, 0, climb_rate * time) + current_pos;
    Eigen::Vector3d vel = Eigen::Vector3d(cruise_speed_ * std::cos(yaw), -cruise_speed_ * std::sin(yaw), -climb_rate);
    const double roll = std::atan(rate(2) * cruise_speed_ / 9.81);
    const double pitch = std::atan(climb_rate / cruise_speed_);
    Eigen::Vector4d att = rpy2quaternion(roll, -pitch, -yaw);  // TODO: why the hell do you need to reverse signs?

    State state_vector;
    state_vector.position = pos;
    state_vector.velocity = vel;
    state_vector.attitude = att;
    trajectory.states.push_back(state_vector);

    time = time + dt;
  }
  return trajectory;
}

PathSegment ManeuverLibrary::generateCircleTrajectory(Eigen::Vector3d center_pos, double radius, const double dt) {
  PathSegment trajectory;
  trajectory.states.clear();

  /// TODO: Fix sign conventions for curvature
  double horizon = 2 * M_PI * radius / cruise_speed_;
  trajectory.curvature = 1 / radius;
  trajectory.dt = dt;
  for (int i = 0; i < std::max(1.0, horizon / dt); i++) {
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector4d att;

    State state_vector;
    state_vector.position = pos;
    state_vector.velocity = vel;
    state_vector.attitude = att;
    trajectory.states.push_back(state_vector);
  }
  return trajectory;
}

Path ManeuverLibrary::getRandomPrimitive() {
  Path primitive;

  std::shared_ptr<Primitive> primitives = motion_primitive_tree_;
  primitive.appendSegment(primitives->segment);
  while (primitives->has_child()) {
    int i = std::rand() % primitives->child_primitives.size();
    primitives = primitives->child_primitives[i];
    primitive.appendSegment(primitives->segment);
  }

  return primitive;
}

Eigen::Vector3d ManeuverLibrary::getRandomPrimitiveRate() const {
  int i = std::rand() % primitive_rates_.size();
  return primitive_rates_[i];
}

Eigen::Vector3d ManeuverLibrary::setTerrainRelativeGoalPosition(const Eigen::Vector3d &pos) {
  Eigen::Vector3d new_goal = pos;
  double terrain_altitude = terrain_map_->getGridMap().atPosition("elevation", Eigen::Vector2d(pos(0), pos(1)));

  if (pos(2) > 0.0) {  // Update if desired goal altitude is higher than zero
    goal_terrain_altitude_ = pos(2);
  }

  new_goal(2) = terrain_altitude + goal_terrain_altitude_;

  setGoalPosition(new_goal);
  return new_goal;
}

Eigen::Vector4d ManeuverLibrary::rpy2quaternion(double roll, double pitch, double yaw) {
  double cy = std::cos(yaw * 0.5);
  double sy = std::sin(yaw * 0.5);
  double cp = std::cos(pitch * 0.5);
  double sp = std::sin(pitch * 0.5);
  double cr = std::cos(roll * 0.5);
  double sr = std::sin(roll * 0.5);

  Eigen::Vector4d q;
  q(0) = cr * cp * cy + sr * sp * sy;
  q(1) = sr * cp * cy - cr * sp * sy;
  q(2) = cr * sp * cy + sr * cp * sy;
  q(3) = cr * cp * sy - sr * sp * cy;

  q.normalize();

  return q;
}

std::vector<ViewPoint> ManeuverLibrary::sampleViewPointFromPath(Path &segment) {
  std::vector<ViewPoint> viewpoint_vector;
  double sample_freq = 1.0;
  std::vector<Eigen::Vector3d> pos_vector = segment.position();
  std::vector<Eigen::Vector4d> att_vector = segment.attitude();
  double elapsed_time = 0.0;
  double dt = 0.1;
  for (size_t i = 0; i < pos_vector.size(); i++) {
    if (elapsed_time > sample_freq) {
      elapsed_time = 0.0;
      Eigen::Vector3d pos = pos_vector[i];  // TODO: sample from
      Eigen::Vector4d att = att_vector[i];
      /// TODO: Set ID correctly
      ViewPoint viewpoint(i, pos, att);
      viewpoint_vector.push_back(viewpoint);
    }
    elapsed_time += dt;
  }
  return viewpoint_vector;
}

std::vector<ViewPoint> ManeuverLibrary::sampleViewPointFromPathSegment(PathSegment &segment) {
  std::vector<ViewPoint> viewpoint_vector;
  double sample_freq = 1.0;
  std::vector<Eigen::Vector3d> pos_vector = segment.position();
  std::vector<Eigen::Vector4d> att_vector = segment.attitude();
  double elapsed_time = 0.0;
  double dt = 0.1;
  for (size_t i = 0; i < pos_vector.size(); i++) {
    if (elapsed_time > sample_freq) {
      elapsed_time = 0.0;
      Eigen::Vector3d pos = pos_vector[i];  // TODO: sample from
      Eigen::Vector4d att = att_vector[i];
      /// TODO: Set ID correctly
      ViewPoint viewpoint(i, pos, att);
      viewpoint_vector.push_back(viewpoint);
    }
    elapsed_time += dt;
  }
  return viewpoint_vector;
}

#endif
