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
 * 3. Neither the name terrain-navigation nor the names of its contributors may be
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

#ifndef MANEUVER_LIBRARY_H
#define MANEUVER_LIBRARY_H

#include "terrain_planner/maneuver_library.h"
#include <iostream>

ManeuverLibrary::ManeuverLibrary() {
  primitive_rates_.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, 3.0, 0.0));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, -1.5, -0.0));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, 0.0, 0.3));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, 0.0, -0.3));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, 3.0, 0.3));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, -1.5, -0.3));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, 0.0, 0.15));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, 0.0, -0.15));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, 3.0, 0.15));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, -1.5, -0.15));
}

ManeuverLibrary::~ManeuverLibrary() {}

std::vector<TrajectorySegments> &ManeuverLibrary::generateMotionPrimitives(const Eigen::Vector3d current_pos,
                                                                           const Eigen::Vector3d current_vel,
                                                                           const Eigen::Vector4d current_att,
                                                                           TrajectorySegments &current_path) {
  Trajectory current_segment;
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
  int tree_depth = 2;
  for (int i = 0; i < tree_depth; i++) {
    expandPrimitives(motion_primitive_tree_, primitive_rates_, planning_horizon_);
  }
  // Add emergency primitives at the end of the trajectory
  std::vector<Eigen::Vector3d> emergency_rates;
  emergency_rates.push_back(Eigen::Vector3d(0.0, 0.0, 0.3));
  double horizon = 2 * M_PI / emergency_rates[0](2);
  expandPrimitives(motion_primitive_tree_, emergency_rates, horizon);

  motion_primitives_.clear();
  motion_primitives_ = motion_primitive_tree_->getMotionPrimitives();

  return motion_primitives_;
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
      Trajectory trajectory = generateArcTrajectory(rate, horizon, current_pos, current_vel);
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

bool ManeuverLibrary::checkCollisionsTree(std::shared_ptr<Primitive> &primitive,
                                          std::vector<TrajectorySegments> &valid_primitives, bool check_valid_child) {
  bool valid_segment{true};
  // Check collision with terrain
  Trajectory &current_trajectory = primitive->segment;
  valid_segment = valid_segment && checkTrajectoryCollision(current_trajectory, "distance_surface", true);
  // Check collision with maximum terrain altitude
  if (check_max_altitude_) {
    valid_segment = valid_segment && checkTrajectoryCollision(current_trajectory, "max_elevation", false);
  }
  primitive->validity = valid_segment;

  // Check recursively if there the leaf has a valid child
  // If there are no valid child available, there the leaf is invalidated
  if (primitive->has_child()) {
    // If the primitive segment is valid and has a child it needs to have at least one child that is valid
    bool has_valid_child{false};
    for (auto &child : primitive->child_primitives) {
      std::vector<TrajectorySegments> child_segments;
      if (checkCollisionsTree(child, child_segments)) {
        has_valid_child = true;
        // for (auto &child : child_segments) {
        //   TrajectorySegments valid_segments;
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
  std::vector<TrajectorySegments> valid_primitives;
  bool valid = checkCollisionsTree(motion_primitive_tree_, valid_primitives);
  valid_primitives_ = valid_primitives;
  return valid;
}

std::vector<TrajectorySegments> ManeuverLibrary::checkRelaxedCollisions() {
  // Iterate through all motion primtiives and only return collision free primitives
  // This is assuming that there are no collision free primitives, therefore the violation of the constraint is measured

  std::vector<TrajectorySegments> valid_primitives;
  for (auto &trajectory : motion_primitives_) {
    double distance_surface_violation = getTrajectoryCollisionCost(trajectory, "distance_surface");
    double max_elevation_violation = getTrajectoryCollisionCost(trajectory, "max_elevation", false);
    trajectory.validity = !(max_elevation_violation > 0.0 || distance_surface_violation > 0.0);
    trajectory.utility = (-1.0) * (max_elevation_violation + distance_surface_violation) / 1000.0;
    valid_primitives.push_back(trajectory);
  }
  return valid_primitives;
}

bool ManeuverLibrary::checkTrajectoryCollision(Trajectory &trajectory, const std::string &layer, bool is_above) {
  /// TODO: Reference gridmap terrain
  for (auto position : trajectory.position()) {
    // TODO: Make max terrain optional
    if (terrain_map_->isInCollision(layer, position, is_above)) {
      return false;
    }
  }
  return true;
}

bool ManeuverLibrary::checkTrajectoryCollision(TrajectorySegments &trajectory, const std::string &layer,
                                               bool is_above) {
  /// TODO: Reference gridmap terrain
  for (auto position : trajectory.position()) {
    // TODO: Make max terrain optional
    if (terrain_map_->isInCollision(layer, position, is_above)) {
      return false;
    }
  }
  return true;
}

double ManeuverLibrary::getTrajectoryCollisionCost(TrajectorySegments &trajectory, const std::string &layer,
                                                   bool is_above) {
  double cost{0.0};
  /// Iterate through whole trajectory to calculate collision depth
  for (auto position : trajectory.position()) {
    cost += terrain_map_->getCollisionDepth(layer, position, is_above);
  }
  return cost;
}

std::vector<TrajectorySegments> ManeuverLibrary::AppendSegment(std::vector<TrajectorySegments> &first_segment,
                                                               const std::vector<Eigen::Vector3d> &rates,
                                                               const double horizon) {
  // Append second segment for each primitive
  std::vector<TrajectorySegments> second_segment;
  for (auto trajectory_segments : first_segment) {
    for (auto rate : rates) {
      Eigen::Vector3d end_pos = trajectory_segments.lastSegment().states.back().position;
      Eigen::Vector3d end_vel = trajectory_segments.lastSegment().states.back().velocity;
      Trajectory new_segment = generateArcTrajectory(rate, horizon, end_pos, end_vel);
      TrajectorySegments trajectory_2 = trajectory_segments;
      trajectory_2.appendSegment(new_segment);
      second_segment.push_back(trajectory_2);
    }
  }
  return second_segment;
}

/// TODO: Change rate vector for curvature and climbrates
Trajectory ManeuverLibrary::generateArcTrajectory(Eigen::Vector3d rate, const double horizon,
                                                  Eigen::Vector3d current_pos, Eigen::Vector3d current_vel,
                                                  const double dt) {
  Trajectory trajectory;
  trajectory.states.clear();

  double time = 0.0;
  const double current_yaw = std::atan2(-1.0 * current_vel(1), current_vel(0));
  const double climb_rate = rate(1);
  trajectory.climb_rate = climb_rate;
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

TrajectorySegments ManeuverLibrary::getBestPrimitive() {
  /// TODO: Implement best first search on tree
  TrajectorySegments primitive;
  // Calculate utilities of each primitives
  for (auto &trajectory : valid_primitives_) {
    if (!trajectory.valid()) continue;
    // Calculate goal utility
    Eigen::Vector3d end_pos = trajectory.lastSegment().states.back().position;
    double terrain_altitude = end_pos(3);
    if (terrain_map_->getGridMap().isInside(Eigen::Vector2d(end_pos(0), end_pos(1)))) {
      terrain_altitude =
          end_pos(3) - terrain_map_->getGridMap().atPosition("elevation", Eigen::Vector2d(end_pos(0), end_pos(1)));
    }
    end_pos(3) = terrain_altitude;
    Eigen::Vector3d distance_vector = end_pos - Eigen::Vector3d(goal_pos_(0), goal_pos_(1), goal_pos_(2));
    trajectory.utility += 1 / distance_vector.norm();
  }

  double best_utility{-std::numeric_limits<double>::infinity()};
  int best_index{0};
  if (valid_primitives_.size() > 0) {
    for (size_t k = 0; k < valid_primitives_.size(); k++) {
      if (valid_primitives_[k].utility > best_utility) {
        best_utility = valid_primitives_[k].utility;
        best_index = k;
      }
    }
    return valid_primitives_[best_index];
  }

  return primitive;
}

TrajectorySegments ManeuverLibrary::getRandomPrimitive() {
  TrajectorySegments primitive;

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

std::vector<ViewPoint> ManeuverLibrary::sampleViewPointFromTrajectorySegment(TrajectorySegments &segment) {
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

std::vector<ViewPoint> ManeuverLibrary::sampleViewPointFromTrajectory(Trajectory &segment) {
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
