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

Eigen::Vector4d rpy2quaternion(double roll, double pitch, double yaw) {
  double cy = std::cos(yaw * 0.5);
  double sy = std::sin(yaw * 0.5);
  double cp = std::cos(pitch * 0.5);
  double sp = std::sin(pitch * 0.5);
  double cr = std::cos(roll * 0.5);
  double sr = std::sin(roll * 0.5);

  Eigen::Vector4d q;
  q(0) = cr * cp * cy + sr * sp * sy;
  q(1) = sr * cp * cy - cr * sp * sy;
  q(2) = cr * sp * cy + sr * cp * sy;
  q(3) = cr * cp * sy - sr * sp * cy;

  q.normalize();

  return q;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "adaptive_viewutility");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  std::shared_ptr<AdaptiveViewUtility> adaptive_viewutility = std::make_shared<AdaptiveViewUtility>(nh, nh_private);

  std::string mesh_file_path{};
  std::string output_file_path{};
  nh_private.param<std::string>("terrain_mesh_path", mesh_file_path, "resources/cadastre.tif");
  nh_private.param<std::string>("output_file_path", output_file_path, "output/map_data.csv");
  GeometricPriorSettings setting;
  double view_distance, min_triangulation_angle, sigma_k;
  nh_private.param<double>("min_traingulation_angle", min_triangulation_angle, 0.78);
  nh_private.param<double>("reference_view_distance", view_distance, 150.0);
  nh_private.param<double>("sigma_k", sigma_k, 45.0 / 180.0 * M_PI);
  setting.min_triangulation_angle = min_triangulation_angle;
  setting.reference_view_distance = view_distance;
  setting.sigma_k = sigma_k;

  adaptive_viewutility->getViewUtilityMap()->setGeometricPriorSettings(setting);

  // Add elevation map from GeoTIFF file defined in path
  adaptive_viewutility->LoadMap(mesh_file_path);
  std::cout << "Successfully loaded map" << std::endl;
  bool initialized_time = false;
  ros::Time start_time_ = ros::Time::now();
  double start_time_seconds_ = 0.0;

  // adaptive_viewutility->setCurrentState(vehicle_pos, vehicle_vel);
  Profiler pipeline_perf("Planner Loop");
  pipeline_perf.tic();

  /// set Current state of vehicle
  const double altitude = 150.0;
  const Eigen::Vector3d origin(0.0, 0.0, 0.0);
  const Eigen::Vector3d offset(5.0, 5.0, 0.001);
  Eigen::Vector3d vehicle_pos(0.0, 0.0, altitude);
  Eigen::Vector3d vehicle_vel(0.0, 0.0, 0.0);
  Eigen::Vector4d vehicle_att(1.0, 0.0, 0.0, 0.0);

  vehicle_att = rpy2quaternion(0.0, 30.0 / 180, 0.0);

  double width = 100;
  double height = 100;
  double resolution = 10.0;
  int num_width = int(width / resolution);
  int num_height = int(height / resolution);

  Trajectory reference_trajectory;

  for (int i = 0; i < num_width; i++) {
    for (int j = 0; j < num_height; j++) {
      vehicle_pos << -width * 0.5 + width / num_width * i, -height * 0.5 + height / num_height * j, 0.0;
      vehicle_pos = vehicle_pos + origin;
      State state_vector;
      state_vector.position = vehicle_pos;
      state_vector.velocity = vehicle_vel;
      state_vector.attitude = vehicle_att;
      reference_trajectory.states.push_back(state_vector);
    }
  }

  adaptive_viewutility->UpdateUtility(reference_trajectory);
  adaptive_viewutility->OutputMapData(output_file_path);

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
