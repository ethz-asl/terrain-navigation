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

#ifndef VIEWPLANNER_H
#define VIEWPLANNER_H

#include "adaptive_viewutility/viewplanner.h"
#include <iostream>

ViewPlanner::ViewPlanner() {
  primitive_rates_.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, 0.0, 0.25));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, 0.0, -0.25));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, 3.0, 0.25));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, -3.0, -0.25));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, 3.0, 0.0));
  primitive_rates_.push_back(Eigen::Vector3d(0.0, -3.0, -0.0));
}

ViewPlanner::~ViewPlanner() {}

std::vector<Trajectory> &ViewPlanner::generateMotionPrimitives(const Eigen::Vector3d current_pos,
                                                               const Eigen::Vector3d current_vel) {
  motion_primitives_.clear();

  for (auto rate : primitive_rates_) {
    Trajectory trajectory = generateArcTrajectory(rate, current_pos, current_vel);
    Eigen::Vector3d end_pos = trajectory.states.back().position;
    Eigen::Vector3d end_vel = trajectory.states.back().velocity;
    for (auto rate : primitive_rates_) {
      Trajectory trajectory_1 = trajectory;
      AppendSegment(trajectory_1, rate, end_pos, end_vel);
      Eigen::Vector3d end_pos2 = trajectory_1.states.back().position;
      Eigen::Vector3d end_vel2 = trajectory_1.states.back().velocity;
      for (auto rate : primitive_rates_) {
        Trajectory trajectory_2 = trajectory_1;
        AppendSegment(trajectory_2, rate, end_pos2, end_vel2);
        motion_primitives_.push_back(trajectory_2);
      }
    }
  }

  return motion_primitives_;
}

void ViewPlanner::AppendSegment(Trajectory &trajectory, const Eigen::Vector3d &rate, const Eigen::Vector3d &end_pos,
                                const Eigen::Vector3d &end_vel) {
  Trajectory trajectory_2 = generateArcTrajectory(rate, end_pos, end_vel);
  trajectory.states.insert(trajectory.states.end(), trajectory_2.states.begin(), trajectory_2.states.end());
}

Trajectory ViewPlanner::generateArcTrajectory(Eigen::Vector3d rate, Eigen::Vector3d current_pos,
                                              Eigen::Vector3d current_vel) {
  Trajectory trajectory;
  trajectory.states.clear();

  double time = 0.0;
  const double current_yaw = std::atan2(current_vel(1), current_vel(0));
  const double climb_rate = rate(1);

  for (int i = 0; i < std::max(1.0, planning_horizon_ / sampling_time_); i++) {
    if (std::abs(rate(2)) < 0.0001) {
      rate(2) > 0.0 ? rate(2) = 0.0001 : rate(2) = -0.0001;
    }
    time = time + sampling_time_;
    double yaw = rate(2) * time + current_yaw;
    Eigen::Vector3d pos =
        cruise_speed_ / rate(2) *
            Eigen::Vector3d(std::sin(yaw) - std::sin(current_yaw), std::cos(yaw) - std::cos(current_yaw), 0) +
        Eigen::Vector3d(0, 0, climb_rate * time) + current_pos;
    Eigen::Vector3d vel = cruise_speed_ * Eigen::Vector3d(std::cos(yaw), std::sin(yaw), climb_rate);
    const double roll = std::atan(rate(2) * cruise_speed_ / 9.81);
    const double pitch = std::atan(climb_rate / cruise_speed_);  /// TODO: link pitch to climbrate
    Eigen::Vector4d att = rpy2quaternion(roll, -pitch, -yaw);
    State state_vector;
    state_vector.position = pos;
    state_vector.velocity = vel;
    state_vector.attitude = att;
    trajectory.states.push_back(state_vector);
  }
  return trajectory;
}

Trajectory &ViewPlanner::getBestPrimitive() {
  double best_utility = 0.0;
  int best_index = 0;
  for (int k = 0; k < motion_primitives_.size(); k++) {
    if (motion_primitives_[k].utility > best_utility) {
      best_utility = motion_primitives_[k].utility;
      best_index = k;
    }
  }
  return motion_primitives_[best_index];
}

Trajectory &ViewPlanner::getRandomPrimitive() {
  int i = std::rand() % motion_primitives_.size();
  return motion_primitives_[i];
}

Eigen::Vector4d ViewPlanner::rpy2quaternion(double roll, double pitch, double yaw) {
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
