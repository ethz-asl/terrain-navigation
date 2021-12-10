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

#include <Eigen/Dense>
#include <vector>

struct State {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector4d attitude;
};

struct Trajectory {
  std::vector<State> states;
  double utility{0.0};
};

class ViewPlanner {
 public:
  ViewPlanner();
  virtual ~ViewPlanner();
  std::vector<Trajectory>& generateMotionPrimitives(const Eigen::Vector3d current_pos,
                                                    const Eigen::Vector3d current_vel);
  std::vector<Trajectory>& getMotionPrimitives() { return motion_primitives_; }
  Trajectory& getBestPrimitive();
  Trajectory& getRandomPrimitive();
  Trajectory generateArcTrajectory(Eigen::Vector3d rates, Eigen::Vector3d current_pos, Eigen::Vector3d current_vel);
  static Eigen::Vector4d rpy2quaternion(double roll, double pitch, double yaw);
  void AppendSegment(Trajectory& trajectory, const Eigen::Vector3d& rate, const Eigen::Vector3d& end_pos,
                     const Eigen::Vector3d& end_vel);
  double getPlanningHorizon() { return planning_horizon_; };

 private:
  // Planner configurations
  std::vector<Eigen::Vector3d> primitive_rates_;
  double sampling_time_{2.0};
  double planning_horizon_{10.0};
  double cruise_speed_{15.0};

  std::vector<Trajectory> motion_primitives_;
};
