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
#include "airsim_client/airsim_client.h"
#include "terrain_navigation/profiler.h"
#include "terrain_navigation/visualization.h"

#include <visualization_msgs/Marker.h>

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

// This application is used to align the Airsim and Adaptive Mapping coordinate systems

int main(int argc, char **argv) {
  ros::init(argc, argv, "adaptive_viewutility");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  ros::Publisher viewpoint_pub = nh.advertise<visualization_msgs::Marker>("viewpoint", 1, true);

  bool initialized_time = false;
  ros::Time start_time_ = ros::Time::now();
  double start_time_seconds_ = 0.0;

  std::shared_ptr<AirsimClient> airsim_client = std::make_shared<AirsimClient>();
  std::shared_ptr<AdaptiveViewUtility> adaptive_viewutility = std::make_shared<AdaptiveViewUtility>(nh, nh_private);

  std::string image_directory{""};

  nh_private.param<std::string>("image_directory", image_directory, image_directory);

  std::string file_path;
  nh_private.param<std::string>("file_path", file_path, "");

  adaptive_viewutility->LoadMap(file_path);

  airsim_client->setImageDirectory(image_directory);
  Eigen::Vector3d player_start = airsim_client->getPlayerStart();
  adaptive_viewutility->getViewUtilityMap()->TransformMap(player_start);
  /// set Current state of vehicle
  Eigen::Vector3d vehicle_pos(0.0, 0.0, 0.0);
  Eigen::Vector3d vehicle_vel(15.0, 0.0, 0.0);
  Eigen::Vector4d vehicle_att = rpy2quaternion(0.0, 0.0, 0.0);

  double resolution = 20.0;
  double width = 200;
  double height = 200;
  double altitude = 2 * resolution;
  int num_width = int(width / resolution);
  int num_height = int(height / resolution);
  int num_altitude = int(altitude / resolution);
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector4d attitude{Eigen::Vector4d(1.0, 0.0, 0.0, 0.0)};

  while (true) {
    airsim_client->getPose(position, attitude);
    ViewPoint viewpoint(0, position, attitude);
    visualization_msgs::Marker viewpoint_marker_msg = Viewpoint2MarkerMsg(0, viewpoint);
    viewpoint_pub.publish(viewpoint_marker_msg);
    // Airsim coordinates are in NED
    std::cout << "Vehicle pose" << std::endl;
    std::cout << "   - position: " << position.transpose() << " / attitude: " << attitude.transpose() << std::endl;
    adaptive_viewutility->MapPublishOnce();
    ros::Duration(1.0).sleep();
  }

  ros::spin();
  return 0;
}
