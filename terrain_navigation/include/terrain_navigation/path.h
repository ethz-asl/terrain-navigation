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

#ifndef PATH_H
#define PATH_H

#include "terrain_navigation/path_segment.h"

class Path {
 public:
  Path(){};
  virtual ~Path(){};
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
  void prependSegment(const PathSegment &trajectory) { segments.insert(segments.begin(), trajectory); };
  void appendSegment(const PathSegment &trajectory) { segments.push_back(trajectory); };
  void appendSegment(const Path &trajectory_segments) {
    for (const auto &trajectory : trajectory_segments.segments) {
      appendSegment(trajectory);
    }
  };
  PathSegment firstSegment() { return segments.front(); }
  PathSegment lastSegment() { return segments.back(); }

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

  PathSegment &getCurrentSegment(const Eigen::Vector3d &position) {
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
    for (size_t i = start_idx; i < segments.size(); i++) {
      length += segments[i].getLength();
    }
    return length;
  }

  bool valid() { return validity; }
  double utility{0.0};
  bool validity{false};
  std::vector<PathSegment> segments;

 private:
  double epsilon_{15.0 * 0.2};
};

#endif
