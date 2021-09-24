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
  primitive_rates_.push_back(Eigen::Vector3d(0.0, 0.0, 0.15));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, 0.0, -0.15));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, 3.0, 0.15));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, -3.0, -0.15));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, 3.0, 0.0));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, -3.0, -0.0));
}

ManeuverLibrary::~ManeuverLibrary() {}

std::vector<Trajectory> &ManeuverLibrary::generateMotionPrimitives(const Eigen::Vector3d current_pos,
                                                                   const Eigen::Vector3d current_vel) {
  motion_primitives_.clear();

  /// TODO: Reformulate as recursive
  std::vector<Trajectory> first_segment;
  for (auto rate : primitive_rates_) {
    Trajectory trajectory = generateArcTrajectory(rate, planning_horizon_, current_pos, current_vel);
    first_segment.push_back(trajectory);
  }

  // Append second segment for each primitive
  std::vector<Trajectory> second_segment = AppendSegment(first_segment, primitive_rates_, planning_horizon_);
  std::vector<Trajectory> third_segment = AppendSegment(second_segment, primitive_rates_, planning_horizon_);

  std::vector<Eigen::Vector3d> emergency_rates;
  emergency_rates.push_back(Eigen::Vector3d(0.0, 0.0, 0.15));
  double horizon = 2 * M_PI / emergency_rates[0](2);
  std::vector<Trajectory> fourth_segment = AppendSegment(third_segment, emergency_rates, horizon);

  motion_primitives_ = fourth_segment;
  return motion_primitives_;
}

bool ManeuverLibrary::Solve() {
  valid_primitives_ = checkCollisions();  // TODO: Define minimum distance?
  if (valid_primitives_.size() < 1) {
    return false;  // No valid motion primitive
  }
  /// TODO: Rank primitives
  return true;
}

std::vector<Trajectory> ManeuverLibrary::checkCollisions() {
  // Return only the reference of trajectories
  std::vector<Trajectory> valid_primitives;
  for (auto &trajectory : motion_primitives_) {
    bool no_collision = checkTrajectoryCollision(trajectory);
    if (no_collision) {
      trajectory.validity = true;
      valid_primitives.push_back(trajectory);
    } else {
      trajectory.validity = false;
    }
  }
  return valid_primitives;
}

bool ManeuverLibrary::checkTrajectoryCollision(Trajectory &trajectory) {
  /// TODO: Reference gridmap terrain
  for (auto position : trajectory.position()) {
    // TODO: Make max terrain optional
    if (terrain_map_->isInCollision("distance_surface", position) ||
        !terrain_map_->isInCollision("max_elevation", position)) {
      return false;
    }
  }
  return true;
}

std::vector<Trajectory> ManeuverLibrary::AppendSegment(std::vector<Trajectory> &first_segment,
                                                       const std::vector<Eigen::Vector3d> &rates,
                                                       const double horizon) {
  // Append second segment for each primitive
  std::vector<Trajectory> second_segment;
  for (auto trajectory : first_segment) {
    for (auto rate : rates) {
      Eigen::Vector3d end_pos = trajectory.states.back().position;
      Eigen::Vector3d end_vel = trajectory.states.back().velocity;
      Trajectory new_segment = generateArcTrajectory(rate, horizon, end_pos, end_vel);
      Trajectory trajectory_2 = trajectory;
      trajectory_2.states.insert(trajectory_2.states.end(), new_segment.states.begin(), new_segment.states.end());
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
    Eigen::Vector3d vel = cruise_speed_ * Eigen::Vector3d(std::cos(yaw), -std::sin(yaw), climb_rate);
    const double roll = std::atan(rate(2) * cruise_speed_ / 9.81);
    const double pitch = std::atan(climb_rate / cruise_speed_);  /// TODO: link pitch to climbrate
    Eigen::Vector4d att = rpy2quaternion(roll, -pitch, -yaw);    // TODO: why the hell do you need to reverse signs?
    State state_vector;
    state_vector.position = pos;
    state_vector.velocity = vel;
    state_vector.attitude = att;
    trajectory.states.push_back(state_vector);
  }
  return trajectory;
}

Trajectory &ManeuverLibrary::getBestPrimitive() {
  // Calculate utilities of each primitives
  for (auto &trajectory : valid_primitives_) {
    Eigen::Vector3d end_pos = trajectory.states.back().position;
    Eigen::Vector3d distance_vector = end_pos - goal_pos_;
    trajectory.utility = 1 / distance_vector.norm();
  }

  double best_utility = 0.0;
  int best_index = 0;
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

Trajectory &ManeuverLibrary::getRandomPrimitive() {
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
