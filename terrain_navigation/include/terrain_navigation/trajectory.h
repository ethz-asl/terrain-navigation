/****************************************************************************
 *
 *   Copyright (c) 2021-2022 Jaeyoung Lim. All rights reserved.
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
        length = 2 * M_PI * (1 / curvature);
      } else {
        Eigen::Vector2d segment_start_2d = segment_start.head(2);
        Eigen::Vector2d segment_start_tangent_2d = (states.front().velocity).normalized().head(2);
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
      Eigen::Vector2d segment_start_tangent_2d = segment_start_tangent.head(2);
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
        arc_center = getArcCenter(segment_start_2d, segment_start_tangent_2d, curvature);
        theta = getArcProgress(arc_center, position_2d, segment_start_2d, segment_end_2d, curvature);
      }
      Eigen::Vector2d closest_point_2d = std::abs(1 / curvature) * (position_2d - arc_center).normalized() + arc_center;
      closest_point = Eigen::Vector3d(closest_point_2d(0), closest_point_2d(1),
                                      theta * segment_end(2) + (1 - theta) * segment_start(2));
      Eigen::Vector2d error_vector = (closest_point_2d - arc_center).normalized();  // Position to error vector
      tangent = Eigen::Vector3d((curvature / std::abs(curvature)) * -error_vector(1),
                                (curvature / std::abs(curvature)) * error_vector(0), 0.0);
    }
    /// TODO: This is a workaround for the TECS height tracking
    double altitude_correction = K_z_ * (position(2) - closest_point(2));
    tangent(2) = std::min(std::max(altitude_correction - climb_rate, max_climb_rate_control_), max_sink_rate_control_);
    return theta;
  }

  std::vector<State> states;
  double curvature{0.0};
  double climb_rate{0.0};
  double dt{0.0};
  double utility{0.0};
  double K_z_ = 0.5;
  double max_climb_rate_control_{-3.5};
  double max_sink_rate_control_{2.0};
  bool viewed{false};
  bool reached{false};
  bool is_periodic{false};

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
  std::vector<Eigen::Vector4d> attitude() {
    std::vector<Eigen::Vector4d> attitude_vector;
    for (auto segment : segments) {
      std::vector<Eigen::Vector4d> segment_att = segment.attitude();
      attitude_vector.insert(attitude_vector.end(), segment_att.begin(), segment_att.end());
    }
    return attitude_vector;
  }
  void resetSegments() { segments.clear(); };
  void prependSegment(const Trajectory &trajectory) { segments.insert(segments.begin(), trajectory); };
  void appendSegment(const Trajectory &trajectory) { segments.push_back(trajectory); };
  void appendSegment(const TrajectorySegments &trajectory_segments) {
    for (const auto trajectory : trajectory_segments.segments) {
      appendSegment(trajectory);
    }
  };
  Trajectory firstSegment() { return segments.front(); }
  Trajectory lastSegment() { return segments.back(); }

  /**
   * @brief Get the Closest Point of the current segment
   *
   * @param position
   * @param closest_point
   * @param tangent
   * @param curvature
   * @param epsilon
   */
  void getClosestPoint(const Eigen::Vector3d &position, Eigen::Vector3d &closest_point, Eigen::Vector3d &tangent,
                       double &curvature, double epsilon = 0.001) {
    closest_point = segments.front().states.front().position;

    // Iterate through all segments
    for (auto &segment : segments) {
      if (segment.reached && (&segment != &segments.back())) continue;
      auto theta = segment.getClosestPoint(position, closest_point, tangent, curvature);

      curvature = segment.curvature;
      if (theta <= 0.0) {
        /// theta can be negative on the start of the next segment
        closest_point = segment.states.front().position;
        tangent = (segment.states.front().velocity).normalized();
        return;
      } else if (theta < 1.0) {  /// TODO: This is a magic number in terms of acceptance radius of end of segments
        return;
      } else {
        closest_point = segment.states.back().position;
        tangent = (segment.states.back().velocity).normalized();
        // Consider that the segment is tracked
        segment.reached = true;
      }
    }
    // TODO: Handle the case when all segments are marked as reached
  }

  Eigen::Vector3d getEndofCurrentSegment(const Eigen::Vector3d &position) {
    if (segments.empty())
      return position;
    else
      return getCurrentSegment(position).states.back().position;
  }

  Trajectory &getCurrentSegment(const Eigen::Vector3d &position) {
    double theta{-std::numeric_limits<double>::infinity()};
    Eigen::Vector3d closest_point;
    Eigen::Vector3d tangent;
    double curvature;
    int segment_idx{-1};
    for (auto &segment : segments) {
      segment_idx++;
      if (segment.reached && (&segment != &segments.back())) continue;
      auto theta = segment.getClosestPoint(position, closest_point, tangent, curvature);

      // If current segment is a full circle, and has a next segment, escape when close to start of next segment
      if (segment.is_periodic && (&segment != &segments.back())) {  // Segment is a terminal periodic set
        Eigen::Vector3d next_segment_start = segments[segment_idx].states.front().position;
        if ((closest_point - next_segment_start).norm() < epsilon_) {
          segment.reached = true;
          return segment;
        }
      }

      if (theta <= 0.0) {
        return segment;
      } else if ((theta < 1.0)) {
        return segment;
      } else {
        segment.reached = true;
      }
    }
  }

  int getCurrentSegmentIndex(const Eigen::Vector3d &position, double epsilon = 0.1) {
    Eigen::Vector3d closest_point;
    Eigen::Vector3d tangent;
    double curvature;
    // Iterate through all segments
    int segment_idx{-1};
    for (auto &segment : segments) {
      segment_idx++;
      if (segment.reached && (&segment != &segments.back())) continue;
      auto theta = segment.getClosestPoint(position, closest_point, tangent, curvature);
      if (theta <= 0.0) {
        /// theta can be negative on the start of the next segment
        return segment_idx;
      } else if (theta < 1.0) {  /// TODO: This is a magic number in terms of acceptance radius of end of segments
        return segment_idx;
      }
    }
    return segment_idx;
  }

  /**
   * @brief Get total length of the trajetory segment
   *
   * @param start_idx start index of the segment the total length should be considered (default 0)
   * @return double length of the trajectory segment
   */
  double getLength(const int start_idx = 0) {
    double length{0.0};
    for (int i = start_idx; i < segments.size(); i++) {
      length += segments[i].getLength();
    }
    return length;
  }

  bool valid() { return validity; }
  double utility{0.0};
  bool validity{false};
  std::vector<Trajectory> segments;

 private:
  double epsilon_{15.0 * 0.2};
};

#endif
