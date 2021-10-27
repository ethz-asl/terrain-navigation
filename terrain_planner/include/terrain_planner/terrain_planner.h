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
 * @brief Motion primtiive based Terrain planner library
 *
 * Motion primitive based terrain planner library
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#ifndef TERRAIN_PLANNER_H
#define TERRAIN_PLANNER_H

#include "terrain_planner/common.h"
#include "terrain_planner/maneuver_library.h"
#include "terrain_planner/profiler.h"

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

#include <Eigen/Dense>

enum class SETPOINT_MODE { STATE, PATH };

class TerrainPlanner {
 public:
  TerrainPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
  virtual ~TerrainPlanner();

 private:
  void cmdloopCallback(const ros::TimerEvent &event);
  void statusloopCallback(const ros::TimerEvent &event);
  void publishTrajectory(std::vector<Eigen::Vector3d> trajectory);
  void mavposeCallback(const geometry_msgs::PoseStamped &msg);
  void mavtwistCallback(const geometry_msgs::TwistStamped &msg);
  void MapPublishOnce();
  void publishPoseHistory();
  void publishCandidateManeuvers(const std::vector<Trajectory> &candidate_maneuvers);
  void publishPositionSetpoints(const Eigen::Vector3d &position, const Eigen::Vector3d &velocity);
  void publishPathSetpoints(const Eigen::Vector3d &position, const Eigen::Vector3d &velocity);
  void publishVehiclePose(const Eigen::Vector3d &position, const Eigen::Vector4d &attitude);
  void processSetPoseFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher vehicle_path_pub_;
  ros::Publisher grid_map_pub_;
  ros::Publisher vehicle_pose_pub_;
  ros::Publisher posehistory_pub_;
  ros::Publisher candidate_manuever_pub_;
  ros::Publisher position_setpoint_pub_;
  ros::Publisher position_target_pub_;
  ros::Publisher path_target_pub_;
  ros::Subscriber mavpose_sub_;
  ros::Subscriber mavtwist_sub_;
  ros::Timer cmdloop_timer_, statusloop_timer_;
  ros::Time plan_time_;
  interactive_markers::InteractiveMarkerServer marker_server_;
  visualization_msgs::InteractiveMarker set_goal_marker_;
  Eigen::Vector3d goal_pos_{Eigen::Vector3d(0.0, 0.0, 20.0)};

  SETPOINT_MODE setpoint_mode_{SETPOINT_MODE::STATE};

  std::shared_ptr<ManeuverLibrary> maneuver_library_;
  std::shared_ptr<Profiler> planner_profiler_;
  Trajectory reference_primitive_;

  std::vector<Eigen::Vector3d> vehicle_position_history_;
  std::vector<geometry_msgs::PoseStamped> posehistory_vector_;
  Eigen::Vector3d vehicle_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vehicle_velocity_{Eigen::Vector3d::Zero()};
  Eigen::Vector4d vehicle_attitude_{Eigen::Vector4d(1.0, 0.0, 0.0, 0.0)};
};

#endif
