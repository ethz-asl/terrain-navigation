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
 * @brief Node for testing airsim interface
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include "adaptive_viewutility/adaptive_viewutility.h"
#include "terrain_navigation/profiler.h"
#include "airsim_client/airsim_client.h"

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

  bool initialized_time = false;
  ros::Time start_time_ = ros::Time::now();
  double start_time_seconds_ = 0.0;

  std::shared_ptr<AirsimClient> airsim_client = std::make_shared<AirsimClient>();

  std::string image_directory{""};
  double origin_x, origin_y;
  double origin_z{150.0};
  nh_private.param<std::string>("image_directory", image_directory, image_directory);
  nh_private.param<double>("origin_x", origin_x, origin_x);
  nh_private.param<double>("origin_y", origin_y, origin_y);
  nh_private.param<double>("origin_z", origin_z, origin_z);

  airsim_client->setImageDirectory(image_directory);
  /// set Current state of vehicle
  const Eigen::Vector3d origin(origin_x, origin_y, origin_z);
  Eigen::Vector3d vehicle_pos(0.0, 0.0, 0.0);
  Eigen::Vector3d vehicle_vel(15.0, 0.0, 0.0);
  Eigen::Vector4d vehicle_att = rpy2quaternion(0.0, 0.0 / 180 * M_PI, 0.0);
  double resolution = 20.0;
  double width = 200;
  double height = 200;
  double altitude = resolution;
  int num_width = int(width / resolution);
  int num_height = int(height / resolution);
  int num_altitude = int(altitude / resolution);

  Trajectory reference_trajectory;

  for (int i = 0; i < num_width; i++) {
    for (int j = 0; j < num_height; j++) {
      for (int k = 0; k < num_altitude; k++) {
        vehicle_pos << -width * 0.5 + width / num_width * i, -height * 0.5 + height / num_height * j,
            altitude / num_altitude * k;
        vehicle_pos = vehicle_pos + origin;
        State state_vector;
        state_vector.position = vehicle_pos;
        state_vector.velocity = vehicle_vel;
        state_vector.attitude = vehicle_att;
        reference_trajectory.states.push_back(state_vector);
      }
    }
  }

  for (size_t i = 0; i < reference_trajectory.states.size(); i++) {
    airsim_client->setPose(reference_trajectory.states[i].position, reference_trajectory.states[i].attitude);
    ros::Duration(0.5).sleep();
  }

  ros::Duration(2.0).sleep();

  ros::spin();
  return 0;
}
