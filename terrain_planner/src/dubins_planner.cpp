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
 * @brief Dubins planner for shortest path calculations
 *        Implementation of
 * https://gieseanw.wordpress.com/2012/10/21/a-comprehensive-step-by-step-tutorial-to-computing-dubins-paths/
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include "terrain_planner/dubins_planner.h"

#include <iostream>

DubinsPlanner::DubinsPlanner() {}

DubinsPlanner::~DubinsPlanner() {}

double DubinsPlanner::calculateCSCDistance(Eigen::Vector2d start, double start_heading, int start_direction,
                                           Eigen::Vector2d goal, double goal_heading, int goal_direction,
                                           TrajectorySegments &path) {
  path.resetSegments();
  Eigen::Vector2d start_circle_center = getArcCenter(start, start_heading, minimum_turning_radius, start_direction);
  Eigen::Vector2d goal_circle_center = getArcCenter(goal, goal_heading, minimum_turning_radius, goal_direction);

  /// TODO: Get tangent between the two circles
  Eigen::Vector2d tangent_vector_start;
  Eigen::Vector2d tangent_vector_end;
  getTangent(start_circle_center, start_direction, goal_circle_center, goal_direction, tangent_vector_start,
             tangent_vector_end);
  double tangentlength = (tangent_vector_start - tangent_vector_end).norm();
  double arclength_start =
      getArcLength(start, tangent_vector_start, start_circle_center, minimum_turning_radius, start_direction);
  double arclength_goal =
      getArcLength(tangent_vector_end, goal, goal_circle_center, minimum_turning_radius, goal_direction);

  double distance = arclength_start + tangentlength + arclength_goal;

  Trajectory first_arc_segment =
      getArcTrajectory(start, start_heading, arclength_start, minimum_turning_radius, start_direction);
  path.appendSegment(first_arc_segment);

  Trajectory tangent_segment = getLineTrajectory(tangent_vector_start, tangent_vector_end);
  path.appendSegment(tangent_segment);
  Eigen::Vector2d tangent_vector = tangent_vector_end - tangent_vector_start;
  double tangent_heading = std::atan2(tangent_vector(1), tangent_vector(0));
  Trajectory second_arc_segment =
      getArcTrajectory(tangent_vector_end, tangent_heading, arclength_goal, minimum_turning_radius, goal_direction);
  path.appendSegment(second_arc_segment);

  return distance;
}

double DubinsPlanner::calculateCCCDistance(Eigen::Vector2d start, double start_heading, int start_direction,
                                           Eigen::Vector2d goal, double goal_heading, TrajectorySegments &path) {
  path.resetSegments();

  Eigen::Vector2d start_circle_center = getArcCenter(start, start_heading, minimum_turning_radius, start_direction);
  Eigen::Vector2d goal_circle_center = getArcCenter(goal, goal_heading, minimum_turning_radius, start_direction);
  double D = (goal_circle_center - start_circle_center).norm();
  Eigen::Vector2d v1 = (goal_circle_center - start_circle_center).normalized();
  double theta = std::atan2(v1(1), v1(0)) + start_direction * std::acos(D / (4.0 * minimum_turning_radius));
  Eigen::Vector2d third_circle_center =
      start_circle_center + 2.0 * minimum_turning_radius * Eigen::Vector2d(std::cos(theta), std::sin(theta));

  Eigen::Vector2d pt1 =
      start_circle_center + minimum_turning_radius * (third_circle_center - start_circle_center).normalized();
  Eigen::Vector2d pt2 =
      goal_circle_center + minimum_turning_radius * (third_circle_center - goal_circle_center).normalized();
  double arclength_start = getArcLength(start, pt1, start_circle_center, minimum_turning_radius, start_direction);
  double arclength_middle = getArcLength(pt1, pt2, third_circle_center, minimum_turning_radius, -start_direction);
  double arclength_goal = getArcLength(pt2, goal, goal_circle_center, minimum_turning_radius, start_direction);
  double distance = arclength_start + arclength_middle + arclength_goal;

  Trajectory first_arc_segment =
      getArcTrajectory(start, start_heading, arclength_start, minimum_turning_radius, start_direction);
  path.appendSegment(first_arc_segment);
  double pt1_heading = start_heading + start_direction * arclength_start / minimum_turning_radius;
  Trajectory middle_arc_segment =
      getArcTrajectory(pt1, pt1_heading, arclength_middle, minimum_turning_radius, -start_direction);
  path.appendSegment(middle_arc_segment);
  double pt2_heading = pt1_heading - start_direction * arclength_middle / minimum_turning_radius;
  Trajectory end_arc_segment =
      getArcTrajectory(pt2, pt2_heading, arclength_goal, minimum_turning_radius, start_direction);
  path.appendSegment(end_arc_segment);
  return distance;
}

double DubinsPlanner::getBestCSCPath(const Eigen::Vector2d &start, const Eigen::Vector2d &end,
                                     TrajectorySegments &best_path) {
  double distance = std::numeric_limits<double>::infinity();

  TrajectorySegments candidate_path;
  double rsr_distance = calculateCSCDistance(start, start_heading_, -1, end, goal_heading_, -1, candidate_path);
  if (rsr_distance < distance) {
    distance = rsr_distance;
    best_path = candidate_path;
  }
  double lsl_distance = calculateCSCDistance(start, start_heading_, 1, end, goal_heading_, 1, candidate_path);
  if (lsl_distance < distance) {
    distance = lsl_distance;
    best_path = candidate_path;
  }
  double lsr_distance = calculateCSCDistance(start, start_heading_, 1, end, goal_heading_, -1, candidate_path);
  if (lsr_distance < distance) {
    distance = lsr_distance;
    best_path = candidate_path;
  }
  double rsl_distance = calculateCSCDistance(start, start_heading_, -1, end, goal_heading_, 1, candidate_path);
  if (rsl_distance < distance) {
    distance = rsl_distance;
    best_path = candidate_path;
  }
  return distance;
}

double DubinsPlanner::getBestCCCPath(const Eigen::Vector2d &start, const Eigen::Vector2d &end,
                                     TrajectorySegments &best_path) {
  // find the relative angle for L and right
  TrajectorySegments candidate_path;
  double theta = 0.0;
  double distance = std::numeric_limits<double>::infinity();
  double rlr_distance = calculateCCCDistance(start, start_heading_, -1, end, goal_heading_, candidate_path);
  if (rlr_distance < distance) {
    distance = rlr_distance;
    best_path = candidate_path;
  }
  double lrl_distance = calculateCCCDistance(start, start_heading_, 1, end, goal_heading_, candidate_path);
  if (lrl_distance < distance) {
    distance = lrl_distance;
    best_path = candidate_path;
  }

  return distance;
}

TrajectorySegments DubinsPlanner::Solve() {
  TrajectorySegments shortest_path;
  Eigen::Vector2d start_pos_2d(start_pos_(0), start_pos_(1));
  Eigen::Vector2d goal_pos_2d(goal_pos_(0), goal_pos_(1));
  double euclidean_distance = (start_pos_2d - goal_pos_2d).norm();
  TrajectorySegments best_csc_path;
  double min_distance = getBestCSCPath(start_pos_2d, goal_pos_2d, best_csc_path);
  shortest_path = best_csc_path;

  if (euclidean_distance < 4.0 * minimum_turning_radius) {
    TrajectorySegments best_ccc_path;
    double best_ccc_distance = getBestCCCPath(start_pos_2d, goal_pos_2d, best_ccc_path);
    if (best_ccc_distance < min_distance) {
      min_distance = best_ccc_distance;
      shortest_path = best_ccc_path;
    }
  }
  /// TODO: Solve for climbrate using dubins metric
  return shortest_path;
}
