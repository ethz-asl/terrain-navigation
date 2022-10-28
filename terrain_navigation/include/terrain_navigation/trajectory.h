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
    double theta = angle_pos / psi;
    return theta;
  }

  std::vector<State> states;
  double curvature{0.0};
  double climb_rate{0.0};
  double dt{0.0};
  double utility{0.0};
  bool viewed{false};
  bool reached{false};

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
  Trajectory lastSegment() { return segments.back(); }
  void getClosestPoint(const Eigen::Vector3d &position, Eigen::Vector3d &closest_point, Eigen::Vector3d &tangent,
                       double &curvature, double epsilon = 0.001) {
    double theta{-std::numeric_limits<double>::infinity()};
    closest_point = segments.front().states.front().position;

    // Iterate through all segments
    int segment_idx{-1};
    for (auto &segment : segments) {
      segment_idx++;
      if (segment.reached && (&segment != &segments.back())) continue;
      Eigen::Vector3d segment_start = segment.states.front().position;
      Eigen::Vector3d segment_start_tangent = (segment.states.front().velocity).normalized();
      Eigen::Vector3d segment_end = segment.states.back().position;
      if (segment.states.size() == 1) {
        // Segment only contains a single state, meaning that it is nor a line or a arc
        theta = 1.0;
      } else if (std::abs(segment.curvature) < 0.0001) {
        // Compute closest point on a line segment
        // Get Path Progress
        theta = segment.getLineProgress(position, segment_start, segment_end);
        tangent = (segment_end - segment_start).normalized();
        closest_point = theta * (segment_end - segment_start) + segment_start;
      } else {
        // Compute closest point on a Arc segment
        Eigen::Vector2d position_2d(position(0), position(1));
        Eigen::Vector2d segment_start_2d = segment_start.head(2);
        Eigen::Vector2d segment_start_tangent_2d = segment_start_tangent.head(2);
        Eigen::Vector2d segment_end_2d = segment_end.head(2);
        Eigen::Vector2d arc_center{Eigen::Vector2d::Zero()};
        if ((segment_start_2d - segment_end_2d).norm() < epsilon) {
          arc_center = segment.getArcCenter(segment_start_2d, segment_start_tangent_2d, segment.curvature);
          Eigen::Vector2d start_vector = (segment_start_2d - arc_center).normalized();
          Eigen::Vector2d position_vector = position_2d - arc_center;
          double angle_pos =
              std::atan2(position_vector(1), position_vector(0)) - std::atan2(start_vector(1), start_vector(0));
          wrap_2pi(angle_pos);
          /// TODO: Check for the case for a helix!
          theta = angle_pos / (2 * M_PI);
          if (std::abs(segment_end(2) - segment_start(2)) > 0.01) {
            double theta_altitude = std::max(
                std::min((position(2) - segment_start(2)) / std::abs(segment_end(2) - segment_start(2)), 1.0), 0.0);
            // theta = std::min(theta, theta_altitude);
          }
        } else {
          arc_center = segment.getArcCenter(segment_start_2d, segment_start_tangent_2d, segment.curvature);
          theta = segment.getArcProgress(arc_center, position_2d, segment_start_2d, segment_end_2d, segment.curvature);
        }
        Eigen::Vector2d closest_point_2d =
            std::abs(1 / segment.curvature) * (position_2d - arc_center).normalized() + arc_center;
        closest_point = Eigen::Vector3d(closest_point_2d(0), closest_point_2d(1),
                                        theta * segment_end(2) + (1 - theta) * segment_start(2));
        Eigen::Vector2d error_vector = (closest_point_2d - arc_center).normalized();  // Position to error vector
        tangent = Eigen::Vector3d((segment.curvature / std::abs(segment.curvature)) * -error_vector(1),
                                  (segment.curvature / std::abs(segment.curvature)) * error_vector(0), 0.0);

        /// If current segment is a full circle, and has a next segment, escape when close to start of next segment
        if ((segment_start_2d - segment_end_2d).norm() < epsilon) {
          if (&segment != &segments.back()) {  // Segment is a full circle
            /// TODO: Get next segment from iterator
            Eigen::Vector3d next_segment_start = segments[segment_idx].states.front().position;
            if ((closest_point - next_segment_start).norm() < epsilon) {
              segment.reached = true;
              return;
            }
          }
        }
      }

      double altitude_correction = K_z_ * (position(2) - closest_point(2));
      tangent(2) =
          std::min(std::max(altitude_correction - segment.climb_rate, max_climb_rate_control_), max_sink_rate_control_);
      curvature = segment.curvature;
      if (theta <= 0.0) {
        /// theta can be negative on the start of the next segment
        closest_point = segment_start;
        tangent = (segment.states.front().velocity).normalized();
        return;
      } else if (theta < 1.0) {  /// TODO: This is a magic number in terms of acceptance radius of end of segments
        return;
      } else {
        closest_point = segment_end;
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
    for (auto &segment : segments) {
      if (segment.reached && (&segment != &segments.back())) continue;
      Eigen::Vector3d segment_start = segment.states.front().position;
      Eigen::Vector3d segment_start_tangent = (segment.states.front().velocity).normalized();
      Eigen::Vector3d segment_end = segment.states.back().position;
      if (segment.states.size() == 1) {
        // Segment only contains a single state, meaning that it is nor a line or a arc
        theta = 1.0;
      } else if (std::abs(segment.curvature) < 0.0001) {
        // Compute closest point on a line segment
        // Get Path Progress
        theta = segment.getLineProgress(position, segment_start, segment_end);
      } else {
        // Compute closest point on a Arc segment
        Eigen::Vector2d position_2d(position(0), position(1));
        Eigen::Vector2d segment_start_2d(segment_start(0), segment_start(1));
        Eigen::Vector2d segment_start_tangent_2d = segment_start_tangent.head(2);
        Eigen::Vector2d segment_end_2d(segment_end(0), segment_end(1));
        Eigen::Vector2d arc_center{Eigen::Vector2d::Zero()};
        if ((segment_start_2d - segment_end_2d).norm() < epsilon_) {
          // This is a special logic to handle when the arc segment start and end position is identical
          Eigen::Vector3d rotational_vector(0.0, 0.0, segment.curvature / std::abs(segment.curvature));
          Eigen::Vector3d arc_center_3d =
              segment_start + (1 / std::abs(segment.curvature)) * segment_start_tangent.cross(rotational_vector);

          arc_center = Eigen::Vector2d(arc_center_3d(0), arc_center_3d(1));
          Eigen::Vector2d start_vector = (segment_start_2d - arc_center).normalized();
          Eigen::Vector2d position_vector = position_2d - arc_center;
          double angle_pos =
              std::atan2(position_vector(1), position_vector(0)) - std::atan2(start_vector(1), start_vector(0));
          wrap_2pi(angle_pos);
          theta = angle_pos / (2 * M_PI);
        } else {
          arc_center = segment.getArcCenter(segment_start_2d, segment_start_tangent_2d, segment.curvature);
          theta = segment.getArcProgress(arc_center, position_2d, segment_start_2d, segment_end_2d, segment.curvature);
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

  bool valid() { return validity; }
  double utility{0.0};
  bool validity{false};
  std::vector<Trajectory> segments;

 private:
  double K_z_ = 0.5;
  double max_climb_rate_control_{-3.5};
  double max_sink_rate_control_{2.0};
  double epsilon_{15.0 * 0.1};
};

#endif
