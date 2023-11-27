/****************************************************************************
 *
 *   Copyright (c) 2022-2023 Jaeyoung Lim, Autonomous Systems Lab,
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
#ifndef PRIMITIVE_H
#define PRIMITIVE_H

#include "terrain_navigation/path.h"

class Primitive {
 public:
  Primitive(PathSegment &trajectory) { segment = trajectory; };
  virtual ~Primitive(){};
  Eigen::Vector3d getEndofSegmentPosition() { return segment.states.back().position; }
  Eigen::Vector3d getEndofSegmentVelocity() { return segment.states.back().velocity; }
  bool valid() { return validity; }
  bool has_child() { return !child_primitives.empty(); }

  /**
   * @brief Check if primitive has valid child
   *
   * @return true  Primitive has a child and at least one child is valid
   * @return false Primitive does not have a child or does not have a valid child
   */
  bool has_validchild() {
    if (!has_child()) {
      return false;  // Return false if there is no child
    } else {
      for (auto &child : child_primitives) {
        if (child->valid()) {
          return true;
        }
      }
      return false;
    }
  }

  /**
   * @brief Get the Motion Primitives object
   *
   * @return std::vector<Path>
   */
  std::vector<Path> getMotionPrimitives() {
    std::vector<Path> all_primitives;
    if (has_child()) {
      //! @todo(srmainwaring) remove set but unused variable
      // int i = 0;
      for (const auto &child : child_primitives) {
        std::vector<Path> extended_primitives = child->getMotionPrimitives();
        // Append current segment
        for (auto &primitive : extended_primitives) {
          primitive.prependSegment(segment);
          primitive.validity = validity && primitive.validity;
          all_primitives.push_back(primitive);
        }
        //! @todo(srmainwaring) remove set but unused variable
        // i++;
      }
    } else {  // Append primitive segments
      Path trajectory_segments;
      trajectory_segments.appendSegment(segment);
      trajectory_segments.validity = validity;
      trajectory_segments.utility = (visits < 1) ? 0.0 : utility / visits;
      all_primitives.push_back(trajectory_segments);
    }
    return all_primitives;
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
    return nullptr;
  }

  std::shared_ptr<Primitive> copy() const {
    auto copied_primitive = std::make_shared<Primitive>(*this);
    copied_primitive->child_primitives.clear();
    for (const auto &child : child_primitives) {
      auto copied_child = child->copy();
      copied_primitive->child_primitives.push_back(copied_child);
    }
    return copied_primitive;
  }
  int depth{0};
  double utility{0.0};
  int visits{0};
  // A primitive is not valid if none of the child primitives are valid
  bool validity{true};
  bool evaluation{false};
  PathSegment segment;
  std::vector<std::shared_ptr<Primitive>> child_primitives;

 private:
};

#endif
