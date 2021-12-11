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
 * @brief Node to test planner in the view utiltiy map
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include "adaptive_viewutility/adaptive_viewutility.h"
#include "terrain_navigation/profiler.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "adaptive_viewutility");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  std::shared_ptr<AdaptiveViewUtility> adaptive_viewutility = std::make_shared<AdaptiveViewUtility>(nh, nh_private);

  std::string tiff_path;
  nh_private.param<std::string>("file_path", tiff_path, "resources/cadastre.tif");

  // Add elevation map from GeoTIFF file defined in path
  adaptive_viewutility->LoadMap(tiff_path);

  bool initialized_time = false;
  ros::Time start_time_ = ros::Time::now();
  double start_time_seconds_ = 0.0;

  /// set Current state of vehicle
  Eigen::Vector3d vehicle_pos(0.0, 0.0, 0.0);
  Eigen::Vector3d vehicle_vel(0.0, 0.0, 0.0);
  Eigen::Vector4d vehicle_att(1.0, 0.0, 0.0, 0.0);

  adaptive_viewutility->setCurrentState(vehicle_pos, vehicle_vel);
  Profiler pipeline_perf("Planner Loop");
  pipeline_perf.tic();

  Trajectory reference_trajectory;
  State state_vector;
  state_vector.position = vehicle_pos;
  state_vector.velocity = vehicle_vel;
  state_vector.attitude = vehicle_att;
  reference_trajectory.states.push_back(state_vector);

  vehicle_pos << 20.0, 00.0, 0.0;
  state_vector.position = vehicle_pos;
  state_vector.velocity = vehicle_vel;
  state_vector.attitude = vehicle_att;
  reference_trajectory.states.push_back(state_vector);

  vehicle_pos << 0.0, 80.0, 0.0;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 1.517;
  vehicle_att = ViewPlanner::rpy2quaternion(roll, pitch, yaw);
  state_vector.position = vehicle_pos;
  state_vector.velocity = vehicle_vel;
  state_vector.attitude = vehicle_att;
  reference_trajectory.states.push_back(state_vector);

  vehicle_pos << 100.0, 100.0, 0.0;
  roll = 0.0;
  pitch = 0.5;
  yaw = 1.517;
  vehicle_att = ViewPlanner::rpy2quaternion(roll, pitch, yaw);
  state_vector.position = vehicle_pos;
  state_vector.velocity = vehicle_vel;
  state_vector.attitude = vehicle_att;
  reference_trajectory.states.push_back(state_vector);

  roll = 0.0;
  pitch = 0.0;
  yaw = 0.0;

  vehicle_pos << 300.0, 100.0, 0.0;
  vehicle_att = ViewPlanner::rpy2quaternion(roll, pitch, yaw);
  state_vector.position = vehicle_pos;
  state_vector.velocity = vehicle_vel;
  state_vector.attitude = vehicle_att;
  reference_trajectory.states.push_back(state_vector);

  vehicle_pos << 300.0, 300.0, 0.0;
  vehicle_att = ViewPlanner::rpy2quaternion(roll, pitch, yaw);
  state_vector.position = vehicle_pos;
  state_vector.velocity = vehicle_vel;
  state_vector.attitude = vehicle_att;
  reference_trajectory.states.push_back(state_vector);

  /// TODO: Separate updating utility vs adding viewpoints
  adaptive_viewutility->UpdateUtility(reference_trajectory);

  pipeline_perf.toc();

  while (true) {
    // Visualize results
    adaptive_viewutility->MapPublishOnce();
    adaptive_viewutility->ViewpointPublishOnce();

    ros::Duration(2.0).sleep();
  }

  ros::spin();
  return 0;
}
