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
 * @brief Adaptive view utility estimation node
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#ifndef ADAPTIVE_VIEWUTILITY_H
#define ADAPTIVE_VIEWUTILITY_H

#include "adaptive_viewutility/common.h"
#include "adaptive_viewutility/viewplanner.h"
#include "adaptive_viewutility/viewutility_map.h"
#include "terrain_navigation/viewpoint.h"

#include <Eigen/Dense>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

class AdaptiveViewUtility {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  AdaptiveViewUtility(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
  virtual ~AdaptiveViewUtility();
  void Init();
  void InitializeVehicleFromMap(Eigen::Vector3d &init_pos, Eigen::Vector3d &init_vel);
  bool AddViewPointFromImage(std::string &image_path);
  void AddViewPoint(const int idx, const double &longitude, const double &latitude, const double &altitude);
  void LoadMap(const std::string &path);
  void MapPublishOnce();
  void ViewpointPublishOnce();
  void NormalPublishOnce();
  void UpdateUtility(ViewPoint &viewpoint);
  void UpdateUtility(Trajectory &trajectory);
  void setCurrentState(const Eigen::Vector3d vehicle_pos, const Eigen::Vector3d vehicle_vel) {
    vehicle_position_ = vehicle_pos;
    vehicle_velocity_ = vehicle_vel;
  };
  void setMapResolution(const double resolution) { target_map_resolution_ = resolution; };
  Trajectory getBestPrimitive();
  bool generateMotionPrimitives();
  std::vector<ViewPoint> &getViewPoints() { return viewpoint_; };
  std::shared_ptr<ViewUtilityMap> &getViewUtilityMap() { return viewutility_map_; };
  std::shared_ptr<ViewPlanner> &getViewPlanner() { return viewplanner_; };
  void Visualize();
  void publishCandidatePaths(std::vector<Trajectory> &motion_primitives);
  void estimateViewUtility();
  void OutputMapData(const std::string &path);
  void publishViewpointHistory();

 private:
  void statusLoopCallback(const ros::TimerEvent &event);
  Eigen::Vector4d quaternion2vector4d(Eigen::Quaterniond quaternion);
  void PublishViewpointImage(ViewPoint &viewpoint);
  visualization_msgs::Marker trajectory2MarkerMsg(Trajectory &trajectory, const int id);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Timer statusloop_timer_;
  ros::CallbackQueue statusloop_queue_;
  std::unique_ptr<ros::AsyncSpinner> statusloop_spinner_;
  ros::Publisher camera_pose_pub_;
  ros::Publisher camera_path_pub_;
  ros::Publisher grid_map_pub_;
  ros::Publisher viewpoint_image_pub_;
  ros::Publisher camera_utility_pub_;
  ros::Publisher vehicle_path_pub_;
  ros::Publisher candidate_path_pub_;
  ros::Publisher normal_marker_pub_;
  ros::Publisher viewpoint_pub_;

  std::vector<ViewPoint> viewpoint_;
  std::shared_ptr<ViewUtilityMap> viewutility_map_;
  std::shared_ptr<ViewPlanner> viewplanner_;

  grid_map::GridMap grid_map_;

  Eigen::Vector3d vehicle_position_;
  Eigen::Vector3d vehicle_velocity_;
  std::vector<Eigen::Vector3d> vehicle_position_history_;
  std::vector<Eigen::Vector4d> vehicle_orientation_history_;
  double target_map_resolution_{2.0};
};
#endif
