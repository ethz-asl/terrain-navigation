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

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <Eigen/Dense>
#include <iostream>
#include <vector>

struct State {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector4d attitude;
};

class Trajectory {
 public:
  Trajectory(){};
  virtual ~Trajectory(){};
  std::vector<Eigen::Vector3d> position() {
    std::vector<Eigen::Vector3d> pos_vector;
    for (auto state : states) {
      pos_vector.push_back(state.position);
    }
    return pos_vector;
  }
  std::vector<Eigen::Vector3d> velocity() {
    std::vector<Eigen::Vector3d> vel_vector;
    for (auto state : states) {
      vel_vector.push_back(state.velocity);
    }
    return vel_vector;
  }
  std::vector<State> states;
  double curvature{0.0};
  double climb_rate{0.0};

 private:
};

class TrajectorySegments {
 public:
  TrajectorySegments(){};
  virtual ~TrajectorySegments(){};
  std::vector<Eigen::Vector3d> position() {
    std::vector<Eigen::Vector3d> pos_vector;
    for (auto segment : segments) {
      std::vector<Eigen::Vector3d> segment_pos = segment.position();
      pos_vector.insert(pos_vector.end(), segment_pos.begin(), segment_pos.end());
    }
    return pos_vector;
  }
  std::vector<Eigen::Vector3d> velocity() {
    std::vector<Eigen::Vector3d> vel_vector;
    for (auto segment : segments) {
      std::vector<Eigen::Vector3d> segment_vel = segment.velocity();
      vel_vector.insert(vel_vector.end(), segment_vel.begin(), segment_vel.end());
    }
    return vel_vector;
  }
  void resetSegments() { segments.clear(); };
  void appendSegment(const Trajectory &trajectory) { segments.push_back(trajectory); };
  Trajectory lastSegment() { return segments.back(); }

  double getLineProgress(const Eigen::Vector3d position, const Eigen::Vector3d &segment_start,
                         const Eigen::Vector3d &segment_end) {
    Eigen::Vector3d progress_vector = (segment_end - segment_start).normalized();
    double segment_length = (segment_end - segment_start).norm();
    Eigen::Vector3d error_vector = position - segment_start;
    // Get Path Progress
    double theta = error_vector.dot(progress_vector) / segment_length;
    return theta;
  }

  static Eigen::Vector2d getArcCenter(const Eigen::Vector2d &segment_start, const Eigen::Vector2d &segment_end,
                                      double curvature) {
    double segment_distance = (segment_end - segment_start).norm();
    double center_distance = std::sqrt(std::pow(1 / curvature, 2) - std::pow(0.5 * segment_distance, 2));
    Eigen::Vector3d rotational_vector(0.0, 0.0, curvature / std::abs(curvature));

    Eigen::Vector2d midpoint_2d = 0.5 * (segment_start + segment_end);
    Eigen::Vector2d distance_vector_2d = (segment_end - segment_start).normalized();
    Eigen::Vector3d distance_vector = Eigen::Vector3d(distance_vector_2d(0), distance_vector_2d(1), 0.0);
    Eigen::Vector3d normal_vector = distance_vector.cross(rotational_vector);
    Eigen::Vector2d normal_vector_2d(normal_vector(0), normal_vector(1));

    Eigen::Vector2d arc_center = midpoint_2d + normal_vector_2d * center_distance;
    return arc_center;
  }

  static double getArcProgress(Eigen::Vector2d &arc_center_2d, const Eigen::Vector2d position_2d,
                               const Eigen::Vector2d &segment_start_2d, const Eigen::Vector2d &segment_end_2d,
                               const double curvature) {
    arc_center_2d = getArcCenter(segment_start_2d, segment_end_2d, curvature);

    Eigen::Vector2d pos_vector = (position_2d - arc_center_2d).normalized();
    Eigen::Vector2d start_vector = (segment_start_2d - arc_center_2d).normalized();
    Eigen::Vector2d end_vector = (segment_end_2d - arc_center_2d).normalized();

    double psi = std::atan2(end_vector(1), end_vector(0)) - std::atan2(start_vector(1), start_vector(0));
    double angle_pos = std::atan2(pos_vector(1), pos_vector(0)) - std::atan2(start_vector(1), start_vector(0));
    while (std::abs(psi) > M_PI) {
      if (psi > 0)
        psi = psi - 2 * M_PI;
      else
        psi = psi + 2 * M_PI;
    }
    while (std::abs(angle_pos) > M_PI) {
      if (angle_pos > 0)
        angle_pos = angle_pos - 2 * M_PI;
      else
        angle_pos = angle_pos + 2 * M_PI;
    }

    double theta = angle_pos / psi;
    return theta;
  }

  void getClosestPoint(const Eigen::Vector3d &position, Eigen::Vector3d &closest_point, Eigen::Vector3d &tangent) {
    double theta{-10.0};
    Eigen::Vector2d arc_center;
    for (auto segment : segments) {
      Eigen::Vector3d segment_start = segment.states.front().position;
      Eigen::Vector3d segment_end = segment.states.back().position;
      if (std::abs(segment.curvature) < 0.0001) {
        // Compute closest point on a line segment
        // Get Path Progress
        theta = getLineProgress(position, segment_start, segment_end);
        tangent = (segment_end - segment_start).normalized();
        closest_point = theta * (segment_end - segment_start) + segment_start;
      } else {
        // Compute closest point on a Arc segment
        Eigen::Vector2d position_2d(position(0), position(1));
        Eigen::Vector2d segment_start_2d(segment_start(0), segment_start(1));
        Eigen::Vector2d segment_end_2d(segment_end(0), segment_end(1));
        theta = getArcProgress(arc_center, position_2d, segment_start_2d, segment_end_2d, segment.curvature);
        Eigen::Vector2d error_vector = (position_2d - arc_center).normalized();
        Eigen::Vector2d closest_point_2d =
            std::abs(1 / segment.curvature) * (position_2d - arc_center).normalized() + arc_center;
        closest_point = Eigen::Vector3d(closest_point_2d(0), closest_point_2d(1),
                                        theta * segment_end(2) + (1 - theta) * segment_start(2));
        tangent = Eigen::Vector3d((segment.curvature / std::abs(segment.curvature)) * error_vector(1),
                                  (segment.curvature / std::abs(segment.curvature)) * -error_vector(0), 0.0);
      }
      if (theta < 0.0) {
        closest_point = segment_start;
        return;
      } else if ((theta < 1.0)) {
        return;
      }
    }
  }
  bool valid() { return validity; }
  double utility{0.0};
  bool validity{false};
  std::vector<Trajectory> segments;

 private:
};

#endif
