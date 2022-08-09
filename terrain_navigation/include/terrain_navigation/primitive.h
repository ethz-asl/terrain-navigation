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
#ifndef PRIMITIVE_H
#define PRIMITIVE_H

#include "terrain_navigation/trajectory.h"

class Primitive {
 public:
  Primitive(Trajectory &trajectory) { segment = trajectory; };
  virtual ~Primitive(){};
  std::vector<std::shared_ptr<Primitive>> child_primitives;
  Eigen::Vector3d getEndofSegmentPosition() { return segment.states.back().position; }
  Eigen::Vector3d getEndofSegmentVelocity() { return segment.states.back().velocity; }
  bool valid() { return validity; }
  bool has_child() { return !child_primitives.empty(); }
  bool has_validchild() {
    if (!has_child()) {
      return true;
    } else {
      for (auto &child : child_primitives) {
        if (child->valid()) return true;
      }
      return false;
    }
  }
  std::vector<TrajectorySegments> getMotionPrimitives() {
    std::vector<TrajectorySegments> all_primitives;
    if (has_child()) {
      std::vector<TrajectorySegments> extended_primitives;
      for (auto &child : child_primitives) {
        extended_primitives = child->getMotionPrimitives();
        // Append current segment
        for (auto &primitive : extended_primitives) {
          primitive.prependSegment(segment);
          all_primitives.push_back(primitive);
        }
      }
      return all_primitives;
    } else {  // Append primitive segments
      TrajectorySegments trajectory_segments;
      trajectory_segments.appendSegment(segment);
      trajectory_segments.validity = validity;
      trajectory_segments.utility = (visits < 1) ? 0.0 : utility / visits;
      all_primitives.push_back(trajectory_segments);
    }
    return all_primitives;
  }

  std::shared_ptr<Primitive> getBestChild() {
    int best_idx{0};
    double best_ucb{-1};
    double c{0.0001};
    for (size_t i = 0; i < child_primitives.size(); i++) {
      std::shared_ptr<Primitive> child = child_primitives[i];
      if (child->visits < 1) continue;
      double upper_confidence_bound =
          child->utility / child->visits + c * std::sqrt(2.0 * std::log(visits) / child->visits);
      if (upper_confidence_bound > best_ucb) {
        best_idx = i;
        best_ucb = upper_confidence_bound;
      }
    }

    /// TODO: Safe guard against empty child
    return child_primitives[best_idx];
  }

  std::shared_ptr<Primitive> getRandomChild() {
    int num_child = child_primitives.size();
    /// TODO: Safe guard against empty child
    return child_primitives[std::rand() % num_child];
  }
  std::shared_ptr<Primitive> getValidChild() {
    for (auto &child : child_primitives) {
      if (child->valid()) return child;
    }
  }
  int depth{0};
  double utility{0.0};
  int visits{0};
  // A primitive is not valid if none of the child primitives are valid
  bool validity{true};
  bool evaluation{false};
  Trajectory segment;

 private:
};

#endif
