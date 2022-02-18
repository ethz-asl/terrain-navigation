/****************************************************************************
 *
 *   Copyright (c) 2022 Jaeyoung Lim All rights reserved.
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
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include "terrain_navigation/trajectory.h"

#include <Eigen/Dense>

class DubinsPlanner {
 public:
  DubinsPlanner();
  virtual ~DubinsPlanner();
  void setStartPosition(const Eigen::Vector3d &pos, const double &heading) {
    start_pos_ = pos;
    start_heading_ = heading;
  };
  void setGoalPosition(const Eigen::Vector3d &pos, const double &heading) {
    goal_pos_ = pos;
    goal_heading_ = heading;
  };
  TrajectorySegments Solve();

 private:
  /**
   * @brief Get the Arc Center object
   *
   * @param pos
   * @param heading
   * @param radius
   * @param direction
   * @return Eigen::Vector2d
   */
  Eigen::Vector2d getArcCenter(const Eigen::Vector2d pos, double heading, double radius, int direction) {
    Eigen::Vector3d unit_velocity = Eigen::Vector3d(std::cos(heading), std::sin(heading), 0.0);
    Eigen::Vector3d circle_axis = Eigen::Vector3d(0.0, 0.0, direction).normalized();
    Eigen::Vector3d radial_vector = circle_axis.cross(unit_velocity);
    Eigen::Vector2d center = pos + radius * Eigen::Vector2d(radial_vector(0), radial_vector(1));
    return center;
  }

  /**
   * @brief Get the Tangent object
   *
   * @param c1
   * @param direction1
   * @param c2
   * @param direction2
   * @param v1
   * @param v2
   */
  bool getTangent(const Eigen::Vector2d &c1, int direction1, const Eigen::Vector2d &c2, int direction2,
                  Eigen::Vector2d &v1, Eigen::Vector2d &v2) {
    double distance = (c1 - c2).norm();
    // Handle when circles are within together
    if (distance <= minimum_turning_radius) return false;
    Eigen::Vector2d V1 = (c2 - c1).normalized();
    if (direction1 > 0) {  // Start left
      if (direction2 > 0) {
        direction1 = 1;
        direction2 = -1;
      } else {
        direction1 = -1;
        direction2 = -1;
      }
    } else {
      if (direction2 > 0) {
        direction1 = -1;
        direction2 = 1;
      } else {
        direction1 = 1;
        direction2 = 1;
      }
    }
    double r = minimum_turning_radius;
    double c = (r - direction1 * r) / distance;
    if (c * c > 1.0) return false;
    double h = std::sqrt(std::max(0.0, 1.0 - c * c));
    Eigen::Vector2d n(V1(0) * c - direction2 * h * V1(1), V1(1) * c + direction2 * h * V1(0));
    v1 = Eigen::Vector2d(c1(0) + r * n(0), c1(1) + r * n(1));
    v2 = Eigen::Vector2d(c2(0) + direction1 * r * n(0), c2(1) + direction1 * r * n(1));
    return true;
  };

  /**
   * @brief Get the Arc Length object
   *
   * @param start
   * @param end
   * @param center
   * @param radius radius of the arc
   * @param direction arc direction -1: left turn +1: right turn
   * @return double
   */
  double getArcLength(const Eigen::Vector2d &start, const Eigen::Vector2d &end, Eigen::Vector2d center,
                      const double radius, int direction) {
    Eigen::Vector2d v1 = start - center;
    Eigen::Vector2d v2 = end - center;
    double theta = std::atan2(v2(1), v2(0)) - std::atan2(v1(1), v1(0));
    if (std::abs(theta) <= std::numeric_limits<double>::epsilon()) theta = 0.0;
    if (theta < 0 && direction > 0)
      theta += 2.0 * M_PI;  // direction left
    else if (theta > 0 && direction < 0)
      theta -= 2.0 * M_PI;  // direction right
    return std::abs(radius * theta);
  }
  Trajectory getArcTrajectory(const Eigen::Vector2d &start, const double &heading, const double &arclength,
                              const double &radius, const int &direction) {
    Trajectory trajectory;
    double start_yaw = heading;
    double rate = double(direction) / radius;
    for (double time = 0; time < arclength; time += sampling_time) {
      double yaw = rate * time + start_yaw;
      Eigen::Vector3d pos =
          1.0 / rate * Eigen::Vector3d(std::sin(yaw) - std::sin(start_yaw), -std::cos(yaw) + std::cos(start_yaw), 0) +
          Eigen::Vector3d(start(0), start(1), 0.0);
      Eigen::Vector3d vel = Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0.0);
      Eigen::Vector4d att = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
      State state;
      state.position = pos;
      state.velocity = vel;
      state.attitude = att;
      trajectory.states.push_back(state);
      trajectory.curvature = double(direction) / radius;
    }
    return trajectory;
  }
  Trajectory getLineTrajectory(const Eigen::Vector2d &start, const Eigen::Vector2d &end) {
    Eigen::Vector2d tangent_vector = end - start;
    Trajectory trajectory;
    State tangent_start, tangent_end;
    tangent_start.position = Eigen::Vector3d(start(0), start(1), 0.0);
    tangent_start.velocity = Eigen::Vector3d(tangent_vector(0), tangent_vector(1), 0.0);
    tangent_start.attitude = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
    tangent_end.position = Eigen::Vector3d(end(0), end(1), 0.0);
    tangent_end.velocity = Eigen::Vector3d(tangent_vector(0), tangent_vector(1), 0.0);
    tangent_end.attitude = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
    trajectory.states.push_back(tangent_start);
    trajectory.states.push_back(tangent_end);
    trajectory.curvature = 0.0;
    return trajectory;
  }
  double calculateCSCDistance(Eigen::Vector2d start, double start_heading, int start_direction, Eigen::Vector2d goal,
                              double goal_heading, int goal_direction, TrajectorySegments &path);
  double calculateCCCDistance(Eigen::Vector2d start, double start_heading, int start_direction, Eigen::Vector2d goal,
                              double goal_heading, TrajectorySegments &path);
  double getBestCSCPath(const Eigen::Vector2d &start, const Eigen::Vector2d &end, TrajectorySegments &best_path);
  double getBestCCCPath(const Eigen::Vector2d &start, const Eigen::Vector2d &end, TrajectorySegments &best_path);
  Eigen::Vector3d start_pos_{Eigen::Vector3d::Zero()};
  double start_heading_{0.0};
  Eigen::Vector3d goal_pos_{Eigen::Vector3d::Zero()};
  double goal_heading_{0.0};
  double minimum_turning_radius{10.0};
  double sampling_time = 0.05;
};
