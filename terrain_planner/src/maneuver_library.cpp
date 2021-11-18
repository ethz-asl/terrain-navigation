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
  terrain_map_ = std::make_shared<TerrainMap>();
  primitive_rates_.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, 3.0, 0.0));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, -1.5, -0.0));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, 0.0, 0.3));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, 0.0, -0.3));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, 3.0, 0.3));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, -1.5, -0.3));
  // primitive_rates_.push_back(Eigen::Vector3d(0.0, 0.0, 0.15));
  // primitive_rates_.push_back(Eigen::Vector3d(0.0, 0.0, -0.15));
  // primitive_rates_.push_back(Eigen::Vector3d(0.0, 3.0, 0.15));
  // primitive_rates_.push_back(Eigen::Vector3d(0.0, -1.5, -0.15));
}

ManeuverLibrary::~ManeuverLibrary() {}

std::vector<TrajectorySegments> &ManeuverLibrary::generateMotionPrimitives(const Eigen::Vector3d current_pos,
                                                                           const Eigen::Vector3d current_vel,
                                                                           TrajectorySegments &current_path) {
  motion_primitives_.clear();

  /// TODO: Reformulate as recursive
  std::vector<TrajectorySegments> first_segment;

  Trajectory current_segment;
  if (!current_path.segments.empty()) {
    current_segment = current_path.getCurrentSegment(current_pos);
  }

  for (auto rate : primitive_rates_) {
    TrajectorySegments trajectory_segments;
    Trajectory trajectory;
    if (!current_path.segments.empty()) {
      trajectory_segments.appendSegment(current_segment);
      trajectory = generateArcTrajectory(rate, planning_horizon_, current_segment.states.back().position,
                                         current_segment.states.back().velocity);
    } else {
      trajectory = generateArcTrajectory(rate, planning_horizon_, current_pos, current_vel);
    }
    trajectory_segments.appendSegment(trajectory);
    first_segment.push_back(trajectory_segments);
  }

  // Append second segment for each primitive
  std::vector<TrajectorySegments> second_segment = AppendSegment(first_segment, primitive_rates_, planning_horizon_);
  std::vector<TrajectorySegments> third_segment = AppendSegment(second_segment, primitive_rates_, planning_horizon_);

  std::vector<Eigen::Vector3d> emergency_rates;
  emergency_rates.push_back(Eigen::Vector3d(0.0, 0.0, 0.3));
  double horizon = 2 * M_PI / emergency_rates[0](2);
  std::vector<TrajectorySegments> fourth_segment = AppendSegment(third_segment, emergency_rates, horizon);

  motion_primitives_ = fourth_segment;

  /// TODO: Append to current trajectory segment
  return motion_primitives_;
}

bool ManeuverLibrary::Solve() {
  valid_primitives_ = checkCollisions();  // TODO: Define minimum distance?

  if (valid_primitives_.size() < 1) {
    // Try to see if relaxing max altitude fixes the problem
    valid_primitives_ = checkRelaxedCollisions();  // TODO: Define minimum distance?
    if (valid_primitives_.size() < 1) {
      return false;  // No valid motion primitive
    }
  }
  /// TODO: Rank primitives
  return true;
}

std::vector<TrajectorySegments> ManeuverLibrary::checkCollisions() {
  // Iterate through all motion primtiives and only return collision free primitives
  // The returned primitives are all collision free
  std::vector<TrajectorySegments> valid_primitives;
  for (auto &trajectory : motion_primitives_) {
    // Check collision with terrain
    bool no_terrain_collision = checkTrajectoryCollision(trajectory, "distance_surface", true);
    // Check collision with maximum terrain altitude
    bool max_altitude_collision = checkTrajectoryCollision(trajectory, "max_elevation", false);
    if (no_terrain_collision && max_altitude_collision) {
      trajectory.validity = true;
      valid_primitives.push_back(trajectory);
    } else {
      trajectory.validity = false;
    }
  }
  return valid_primitives;
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

Trajectory ManeuverLibrary::generateArcTrajectory(Eigen::Vector3d rate, const double horizon,
                                                  Eigen::Vector3d current_pos, Eigen::Vector3d current_vel) {
  Trajectory trajectory;
  trajectory.states.clear();

  double time = 0.0;
  const double current_yaw = std::atan2(-1.0 * current_vel(1), current_vel(0));
  const double climb_rate = rate(1);
  trajectory.climb_rate = climb_rate;
  trajectory.curvature = rate(2) / cruise_speed_;

  for (int i = 0; i < std::max(1.0, horizon / dt_); i++) {
    if (std::abs(rate(2)) < 0.0001) {
      rate(2) > 0.0 ? rate(2) = 0.0001 : rate(2) = -0.0001;
    }
    time = time + dt_;
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
  }
  return trajectory;
}

TrajectorySegments &ManeuverLibrary::getBestPrimitive() {
  // Calculate utilities of each primitives
  for (auto &trajectory : valid_primitives_) {
    Eigen::Vector3d end_pos = trajectory.lastSegment().states.back().position;
    Eigen::Vector2d distance_vector =
        Eigen::Vector2d(end_pos(0), end_pos(1)) - Eigen::Vector2d(goal_pos_(0), goal_pos_(1));
    trajectory.utility += 1 / distance_vector.norm();
  }

  double best_utility{-std::numeric_limits<double>::infinity()};
  int best_index{0};
  if (valid_primitives_.size() > 0) {
    for (int k = 0; k < valid_primitives_.size(); k++) {
      if (valid_primitives_[k].utility > best_utility) {
        best_utility = valid_primitives_[k].utility;
        best_index = k;
      }
    }
    return valid_primitives_[best_index];
  } else {
    return getRandomPrimitive();
  }
}

TrajectorySegments &ManeuverLibrary::getRandomPrimitive() {
  int i = 0;
  if (valid_primitives_.size() > 0) {
    i = std::rand() % valid_primitives_.size();
    return valid_primitives_[i];
  } else {
    i = std::rand() % motion_primitives_.size();
    return motion_primitives_[i];
  }
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

#endif
