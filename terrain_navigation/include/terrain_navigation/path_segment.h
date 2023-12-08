/****************************************************************************
 *
 *   Copyright (c) 2021-2023 Jaeyoung Lim, Autonomous Systems Lab,
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

#ifndef PATH_SEGMENT_H
#define PATH_SEGMENT_H

#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <vector>

struct State {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector4d attitude;
};

static void wrap_2pi(double &angle) {
  while ((angle < 0.0) || (angle > 2 * M_PI)) {
    if (angle < 0.0) {
      angle += 2 * M_PI;
    } else {
      angle -= 2 * M_PI;
    }
  }
}

static void wrap_pi(double &angle) {
  while (std::abs(angle) > M_PI) {
    if (angle > 0)
      angle = angle - 2 * M_PI;
    else
      angle = angle + 2 * M_PI;
  }
}

class PathSegment {
 public:
  PathSegment(){};
  virtual ~PathSegment(){};
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
  std::vector<Eigen::Vector4d> attitude() {
    std::vector<Eigen::Vector4d> attitude_vector;
    for (auto state : states) {
      attitude_vector.push_back(state.attitude);
    }
    return attitude_vector;
  }

  static Eigen::Vector2d getArcCenter(const Eigen::Vector2d &segment_start,
                                      const Eigen::Vector2d &segment_start_tangent, double curvature) {
    // Segment is full circle!
    Eigen::Vector3d segment_start_tangent_3d(segment_start_tangent(0), segment_start_tangent(1), 0.0);
    Eigen::Vector3d rotational_vector(0.0, 0.0, curvature / std::abs(curvature));

    Eigen::Vector2d arc_center =
        segment_start + (1 / std::abs(curvature)) * rotational_vector.cross(segment_start_tangent_3d).head(2);
    return arc_center;
  }

  static Eigen::Vector2d getArcCenter(double curvature, const Eigen::Vector2d &segment_start,
                                      const Eigen::Vector2d &segment_start_tangent,
                                      const Eigen::Vector2d &segment_end) {
    Eigen::Vector3d rotational_vector(0.0, 0.0, curvature / std::abs(curvature));

    Eigen::Vector2d error_vector = segment_end - segment_start;
    double segment_distance = error_vector.norm();
    double center_distance = std::sqrt(std::pow(1 / curvature, 2) - std::pow(0.5 * segment_distance, 2));
    Eigen::Vector2d midpoint_2d = 0.5 * (segment_start + segment_end);
    Eigen::Vector2d distance_vector_2d = error_vector.normalized();
    Eigen::Vector3d distance_vector = Eigen::Vector3d(distance_vector_2d(0), distance_vector_2d(1), 0.0);
    Eigen::Vector3d normal_vector;
    if (error_vector.dot(segment_start_tangent) > 0.0) {
      normal_vector = -distance_vector.cross(rotational_vector);
    } else {
      normal_vector = distance_vector.cross(rotational_vector);
    }
    Eigen::Vector2d arc_center = midpoint_2d + normal_vector.head(2) * center_distance;
    return arc_center;
  }

  static double getLineProgress(const Eigen::Vector3d position, const Eigen::Vector3d &segment_start,
                                const Eigen::Vector3d &segment_end) {
    Eigen::Vector3d progress_vector = (segment_end - segment_start).normalized();
    double segment_length = (segment_end - segment_start).norm();
    Eigen::Vector3d error_vector = position - segment_start;
    // Get Path Progress
    double theta = error_vector.dot(progress_vector) / segment_length;
    return theta;
  }

  static double getArcProgress(const Eigen::Vector2d &arc_center_2d, const Eigen::Vector2d position_2d,
                               const Eigen::Vector2d &segment_start_2d, const Eigen::Vector2d &segment_end_2d,
                               const double curvature) {
    Eigen::Vector2d pos_vector = (position_2d - arc_center_2d).normalized();
    Eigen::Vector2d start_vector = (segment_start_2d - arc_center_2d).normalized();
    Eigen::Vector2d end_vector = (segment_end_2d - arc_center_2d).normalized();
    /// TODO: if end_vector == start_vector psi = 2_PI
    // Progress does not depend on curvature!
    double psi;
    double angle_pos;
    if (curvature > 0.0) {
      psi = std::atan2(end_vector(1), end_vector(0)) - std::atan2(start_vector(1), start_vector(0));
      angle_pos = std::atan2(pos_vector(1), pos_vector(0)) - std::atan2(start_vector(1), start_vector(0));
    } else {
      psi = -std::atan2(end_vector(1), end_vector(0)) + std::atan2(start_vector(1), start_vector(0));
      angle_pos = -std::atan2(pos_vector(1), pos_vector(0)) + std::atan2(start_vector(1), start_vector(0));
    }
    wrap_2pi(psi);
    wrap_2pi(angle_pos);

    double residual_pi = std::max(2 * M_PI - psi, 0.0);

    if (angle_pos > psi + 0.5 * residual_pi && residual_pi > 0.0) {
      angle_pos = angle_pos - 2 * M_PI;
    }
    double theta = angle_pos / psi;
    return theta;
  }

  double getLength(double epsilon = 0.001) {
    double length{0.0};
    Eigen::Vector3d segment_start = states.front().position;
    Eigen::Vector3d segment_end = states.back().position;
    if (states.size() == 1) {
      return 0.0;
    } else if (std::abs(curvature) < 0.0001) {
      // Segment is a line segment
      length = (segment_end - segment_start).norm();
    } else {
      // Compute closest point on a Arc segment
      Eigen::Vector2d segment_start_2d = segment_start.head(2);
      Eigen::Vector2d segment_end_2d = segment_end.head(2);
      if ((segment_start_2d - segment_end_2d).norm() < epsilon) {
        // Return full circle length
        length = 2 * M_PI * (1 / std::abs(curvature));
      } else {
        Eigen::Vector2d segment_start_2d = segment_start.head(2);
        Eigen::Vector2d segment_start_tangent_2d = (states.front().velocity).head(2).normalized();
        Eigen::Vector2d segment_end_2d = segment_end.head(2);

        Eigen::Vector2d arc_center_2d = getArcCenter(segment_start_2d, segment_start_tangent_2d, curvature);
        Eigen::Vector2d start_vector = (segment_start_2d - arc_center_2d).normalized();
        Eigen::Vector2d end_vector = (segment_end_2d - arc_center_2d).normalized();

        double psi = std::atan2(end_vector(1), end_vector(0)) - std::atan2(start_vector(1), start_vector(0));
        wrap_2pi(psi);
        length = (1 / std::abs(curvature)) * psi;
      }
    }
    return length;
  }

  double getClosestPoint(const Eigen::Vector3d &position, Eigen::Vector3d &closest_point, Eigen::Vector3d &tangent,
                         double &segment_curvature, double epsilon = 0.001) {
    double theta{-std::numeric_limits<double>::infinity()};
    segment_curvature = curvature;
    Eigen::Vector3d segment_start = states.front().position;
    Eigen::Vector3d segment_start_tangent = (states.front().velocity).normalized();
    Eigen::Vector3d segment_end = states.back().position;
    if (states.size() == 1) {
      // Segment only contains a single state, meaning that it is nor a line or a arc
      theta = 1.0;
    } else if (std::abs(curvature) < 0.0001) {
      // Compute closest point on a line segment
      // Get Path Progress
      theta = getLineProgress(position, segment_start, segment_end);
      tangent = (segment_end - segment_start).normalized();
      // Closest point should not be outside segment - constrain theta to [0.0, 1.0]
      closest_point = std::max(std::min(1.0, theta), 0.0) * (segment_end - segment_start) + segment_start;
    } else {
      // Compute closest point on a Arc segment
      Eigen::Vector2d position_2d(position(0), position(1));
      Eigen::Vector2d segment_start_2d = segment_start.head(2);
      Eigen::Vector2d segment_start_tangent_2d = segment_start_tangent.head(2).normalized();
      Eigen::Vector2d segment_end_2d = segment_end.head(2);
      Eigen::Vector2d arc_center{Eigen::Vector2d::Zero()};
      // Handle when it is a full circle
      if (is_periodic) {
        arc_center = getArcCenter(segment_start_2d, segment_start_tangent_2d, curvature);
        Eigen::Vector2d start_vector = (segment_start_2d - arc_center).normalized();
        Eigen::Vector2d position_vector = position_2d - arc_center;
        double angle_pos =
            std::atan2(position_vector(1), position_vector(0)) - std::atan2(start_vector(1), start_vector(0));
        wrap_2pi(angle_pos);
        /// TODO: Check for the case for a helix!
        theta = angle_pos / (2 * M_PI);
      } else {
        // arc_center = getArcCenter(segment_start_2d, segment_start_tangent_2d, curvature);
        arc_center = getArcCenter(curvature, segment_start_2d, segment_start_tangent_2d, segment_end_2d);
        theta = getArcProgress(arc_center, position_2d, segment_start_2d, segment_end_2d, curvature);
      }
      Eigen::Vector2d closest_point_2d = std::abs(1 / curvature) * (position_2d - arc_center).normalized() + arc_center;
      closest_point = Eigen::Vector3d(closest_point_2d(0), closest_point_2d(1),
                                      theta * segment_end(2) + (1 - theta) * segment_start(2));
      Eigen::Vector2d error_vector = (closest_point_2d - arc_center).normalized();  // Position to error vector
      tangent = Eigen::Vector3d((curvature / std::abs(curvature)) * -error_vector(1),
                                (curvature / std::abs(curvature)) * error_vector(0), 0.0);
    }

    tangent(0) = std::cos(flightpath_angle) * tangent(0);
    tangent(1) = std::cos(flightpath_angle) * tangent(1);
    tangent(2) = std::sin(flightpath_angle);
    return theta;
  }

  std::vector<State> states;
  double curvature{0.0};
  double flightpath_angle{0.0};
  double dt{0.0};
  double utility{0.0};
  bool viewed{false};
  bool reached{false};
  bool is_periodic{false};

 private:
};

#endif
