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
 * @brief Terrain Planner
 *
 * Terrain Planner
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include "terrain_planner/terrain_planner.h"

#include <grid_map_msgs/GridMap.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/Trajectory.h>
#include <planner_msgs/NavigationStatus.h>
#include <visualization_msgs/MarkerArray.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <GeographicLib/Geocentric.hpp>

TerrainPlanner::TerrainPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private), marker_server_("goal") {
  vehicle_path_pub_ = nh_.advertise<nav_msgs::Path>("vehicle_path", 1);
  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.1), &TerrainPlanner::cmdloopCallback,
                                   this);  // Define timer for constant loop rate
  statusloop_timer_ = nh_.createTimer(ros::Duration(2.0), &TerrainPlanner::statusloopCallback,
                                      this);  // Define timer for constant loop rate

  grid_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

  posehistory_pub_ = nh_.advertise<nav_msgs::Path>("geometric_controller/path", 10);
  referencehistory_pub_ = nh_.advertise<nav_msgs::Path>("reference/path", 10);
  candidate_manuever_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1, true);
  position_target_pub_ = nh_.advertise<visualization_msgs::Marker>("position_target", 1, true);
  mavstate_sub_ =
      nh_.subscribe("mavros/state", 1, &TerrainPlanner::mavstateCallback, this, ros::TransportHints().tcpNoDelay());
  position_setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1);
  path_target_pub_ = nh_.advertise<mavros_msgs::Trajectory>("mavros/trajectory/generated", 1);
  vehicle_pose_pub_ = nh_.advertise<visualization_msgs::Marker>("vehicle_pose_marker", 1, true);
  planner_status_pub_ = nh_.advertise<planner_msgs::NavigationStatus>("planner_status", 1, true);

  mavpose_sub_ = nh_.subscribe("mavros/local_position/pose", 1, &TerrainPlanner::mavposeCallback, this,
                               ros::TransportHints().tcpNoDelay());
  mavtwist_sub_ = nh_.subscribe("mavros/local_position/velocity_local", 1, &TerrainPlanner::mavtwistCallback, this,
                                ros::TransportHints().tcpNoDelay());
  global_origin_sub_ = nh_.subscribe("mavros/global_position/gp_origin", 1, &TerrainPlanner::mavGlobalOriginCallback,
                                     this, ros::TransportHints().tcpNoDelay());
  nh_private.param<std::string>("terrain_path", map_path_, "resources/cadastre.tif");
  nh_private.param<std::string>("meshresource_path", mesh_resource_path_, "resources/believer.dae");
  maneuver_library_ = std::make_shared<ManeuverLibrary>();
  maneuver_library_->setPlanningHorizon(5.0);

  planner_profiler_ = std::make_shared<Profiler>("planner");

  set_goal_marker_.header.frame_id = "map";
  set_goal_marker_.name = "set_pose";
  set_goal_marker_.scale = 100.0;
  set_goal_marker_.controls.clear();

  constexpr double kSqrt2Over2 = sqrt(2.0) / 2.0;

  // Set up controls: x, y, z, and yaw.
  visualization_msgs::InteractiveMarkerControl control;
  set_goal_marker_.controls.clear();
  control.orientation.w = kSqrt2Over2;
  control.orientation.x = 0;
  control.orientation.y = kSqrt2Over2;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
  control.name = "move plane";
  set_goal_marker_.controls.push_back(control);

  marker_server_.insert(set_goal_marker_);
  marker_server_.setCallback(set_goal_marker_.name, boost::bind(&TerrainPlanner::processSetPoseFeedback, this, _1));
  marker_server_.applyChanges();
}
TerrainPlanner::~TerrainPlanner() {
  // Destructor
}

void TerrainPlanner::cmdloopCallback(const ros::TimerEvent &event) {
  if (!map_initialized_) return;

  // TODO: Get position setpoint based on time
  double time_since_start = (ros::Time::now() - plan_time_).toSec();
  switch (setpoint_mode_) {
    case SETPOINT_MODE::STATE: {
      /// TODO: Find closest point on the segment
      Eigen::Vector3d reference_position;
      Eigen::Vector3d reference_tangent;
      double reference_curvature{0.0};
      reference_primitive_.getClosestPoint(vehicle_position_, reference_position, reference_tangent,
                                           reference_curvature);
      publishPositionSetpoints(reference_position, reference_tangent, reference_curvature);
      if (current_state_.mode == "OFFBOARD")
        publishPositionHistory(referencehistory_pub_, reference_position, referencehistory_vector_);
      break;
    }
    case SETPOINT_MODE::PATH: {
      // publishPathSetpoints(trajectory_position[0], trajectory_velocity[0]);
      break;
    }
  }

  publishVehiclePose(vehicle_position_, vehicle_attitude_);
  publishPositionHistory(posehistory_pub_, vehicle_position_, posehistory_vector_);
}

void TerrainPlanner::statusloopCallback(const ros::TimerEvent &event) {
  if (local_origin_received_ && !map_initialized_) {
    maneuver_library_->setTerrainMap(map_path_);
    map_initialized_ = true;
    return;
  }
  planner_profiler_->tic();
  /// TODO: Plan from next segment
  // Plan from the end of the current segment
  if (current_state_.mode != "OFFBOARD") {
    reference_primitive_.segments.clear();
  }
  // Only run planner in offboard mode
  maneuver_library_->generateMotionPrimitives(vehicle_position_, vehicle_velocity_, reference_primitive_);
  /// TODO: Switch to chrono
  plan_time_ = ros::Time::now();
  bool result = maneuver_library_->Solve();

  if (result) {
    reference_primitive_ = maneuver_library_->getBestPrimitive();
  } else {
    /// TODO: Take failsafe action when no valid primitive is found
    reference_primitive_ = maneuver_library_->getRandomPrimitive();
  }
  double planner_time = planner_profiler_->toc();
  publishCandidateManeuvers(maneuver_library_->getMotionPrimitives());
  publishTrajectory(reference_primitive_.position());
  MapPublishOnce();

  planner_msgs::NavigationStatus msg;
  msg.header.stamp = ros::Time::now();
  msg.planner_time.data = planner_time;
  planner_status_pub_.publish(msg);
}

void TerrainPlanner::publishTrajectory(std::vector<Eigen::Vector3d> trajectory) {
  nav_msgs::Path msg;
  std::vector<geometry_msgs::PoseStamped> posestampedhistory_vector;
  Eigen::Vector4d orientation(1.0, 0.0, 0.0, 0.0);
  for (auto pos : trajectory) {
    posestampedhistory_vector.insert(posestampedhistory_vector.begin(), vector3d2PoseStampedMsg(pos, orientation));
  }
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.poses = posestampedhistory_vector;
  vehicle_path_pub_.publish(msg);
}

void TerrainPlanner::mavposeCallback(const geometry_msgs::PoseStamped &msg) {
  vehicle_position_ = toEigen(msg.pose.position);
  vehicle_attitude_(0) = msg.pose.orientation.w;
  vehicle_attitude_(1) = msg.pose.orientation.x;
  vehicle_attitude_(2) = msg.pose.orientation.y;
  vehicle_attitude_(3) = msg.pose.orientation.z;
}

void TerrainPlanner::mavtwistCallback(const geometry_msgs::TwistStamped &msg) {
  vehicle_velocity_ = toEigen(msg.twist.linear);
  // mavRate_ = toEigen(msg.twist.angular);
}

void TerrainPlanner::MapPublishOnce() {
  maneuver_library_->getGridMap().setTimestamp(ros::Time::now().toNSec());
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(maneuver_library_->getGridMap(), message);
  grid_map_pub_.publish(message);
}

void TerrainPlanner::publishPositionHistory(ros::Publisher &pub, const Eigen::Vector3d &position,
                                            std::vector<geometry_msgs::PoseStamped> &history_vector) {
  int posehistory_window_ = 20000;
  Eigen::Vector4d vehicle_attitude(1.0, 0.0, 0.0, 0.0);
  history_vector.insert(history_vector.begin(), vector3d2PoseStampedMsg(position, vehicle_attitude));
  if (history_vector.size() > posehistory_window_) {
    history_vector.pop_back();
  }

  nav_msgs::Path msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.poses = history_vector;

  pub.publish(msg);
}

void TerrainPlanner::publishCandidateManeuvers(const std::vector<TrajectorySegments> &candidate_maneuvers) {
  visualization_msgs::MarkerArray msg;

  std::vector<visualization_msgs::Marker> marker;
  visualization_msgs::Marker mark;
  mark.action = visualization_msgs::Marker::DELETEALL;
  marker.push_back(mark);
  msg.markers = marker;
  candidate_manuever_pub_.publish(msg);

  std::vector<visualization_msgs::Marker> maneuver_library_vector;
  int i = 0;
  bool visualize_invalid_trajectories = false;
  for (auto maneuver : candidate_maneuvers) {
    if (maneuver.valid() || visualize_invalid_trajectories) {
      maneuver_library_vector.insert(maneuver_library_vector.begin(), trajectory2MarkerMsg(maneuver, i));
    }
    i++;
  }
  msg.markers = maneuver_library_vector;
  candidate_manuever_pub_.publish(msg);
}

void TerrainPlanner::publishPositionSetpoints(const Eigen::Vector3d &position, const Eigen::Vector3d &velocity,
                                              const double curvature) {
  using namespace mavros_msgs;
  // Publishes position setpoints sequentially as trajectory setpoints
  mavros_msgs::PositionTarget msg;
  msg.header.stamp = ros::Time::now();
  msg.coordinate_frame = PositionTarget::FRAME_LOCAL_NED;
  msg.type_mask = PositionTarget::IGNORE_AFX | PositionTarget::IGNORE_AFY | PositionTarget::IGNORE_AFZ;
  msg.position.x = position(0);
  msg.position.y = position(1);
  msg.position.z = position(2);
  msg.velocity.x = velocity(0);
  msg.velocity.y = velocity(1);
  msg.velocity.z = velocity(2);
  msg.yaw_rate = -curvature;

  position_setpoint_pub_.publish(msg);

  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::ARROW;
  marker.header.frame_id = "map";
  marker.id = 0;
  marker.action = visualization_msgs::Marker::DELETEALL;
  position_target_pub_.publish(marker);

  marker.header.stamp = ros::Time::now();
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 20.0;
  marker.scale.y = 4.0;
  marker.scale.z = 4.0;
  marker.color.a = 0.5;  // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.pose.position.x = position(0);
  marker.pose.position.y = position(1);
  marker.pose.position.z = position(2);
  double yaw = std::atan2(velocity(1), velocity(0));
  marker.pose.orientation.w = std::cos(0.5 * yaw);
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = std::sin(0.5 * yaw);

  position_target_pub_.publish(marker);
}

void TerrainPlanner::mavstateCallback(const mavros_msgs::State::ConstPtr &msg) { current_state_ = *msg; }

void TerrainPlanner::publishPathSetpoints(const Eigen::Vector3d &position, const Eigen::Vector3d &velocity) {
  using namespace mavros_msgs;
  // Publishes position setpoints sequentially as trajectory setpoints
  mavros_msgs::PositionTarget msg;
  msg.header.stamp = ros::Time::now();
  msg.coordinate_frame = PositionTarget::FRAME_LOCAL_NED;
  msg.type_mask = PositionTarget::IGNORE_AFX | PositionTarget::IGNORE_AFY | PositionTarget::IGNORE_AFZ;
  msg.position.x = position(0);
  msg.position.y = position(1);
  msg.position.z = position(2);
  msg.velocity.x = velocity(0);
  msg.velocity.y = velocity(1);
  msg.velocity.z = velocity(2);

  /// TODO: Package trajectory segments into trajectory waypoints

  mavros_msgs::Trajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();
  trajectory_msg.type = mavros_msgs::Trajectory::MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS;
  trajectory_msg.point_valid[0] = true;
  trajectory_msg.point_valid[1] = true;
  trajectory_msg.point_1 = msg;
  trajectory_msg.point_2 = msg;
  trajectory_msg.point_3 = msg;
  trajectory_msg.point_4 = msg;
  trajectory_msg.point_5 = msg;

  path_target_pub_.publish(trajectory_msg);
}

void TerrainPlanner::publishVehiclePose(const Eigen::Vector3d &position, const Eigen::Vector4d &attitude) {
  Eigen::Vector4d mesh_attitude =
      quatMultiplication(attitude, Eigen::Vector4d(std::cos(M_PI / 2), 0.0, 0.0, std::sin(M_PI / 2)));
  geometry_msgs::Pose vehicle_pose = vector3d2PoseMsg(position, mesh_attitude);
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "map";
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.ns = "my_namespace";
  marker.mesh_resource = "file://" + mesh_resource_path_;
  marker.scale.x = 10.0;
  marker.scale.y = 10.0;
  marker.scale.z = 10.0;
  marker.color.a = 0.5;  // Don't forget to set the alpha!
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.pose = vehicle_pose;
  vehicle_pose_pub_.publish(marker);
}

void TerrainPlanner::processSetPoseFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  // TODO: Set goal position from menu
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE) {
    set_goal_marker_.pose = feedback->pose;
    Eigen::Vector2d marker_position_2d(set_goal_marker_.pose.position.x, set_goal_marker_.pose.position.y);
    if (maneuver_library_->getGridMap().isInside(marker_position_2d)) {
      double elevation = maneuver_library_->getGridMap().atPosition("elevation", marker_position_2d);
      set_goal_marker_.pose.position.z = elevation + 200.0;
      marker_server_.setPose(set_goal_marker_.name, set_goal_marker_.pose);
    }
    goal_pos_ = toEigen(feedback->pose);
    maneuver_library_->setGoalPosition(goal_pos_);
  }
  marker_server_.applyChanges();
}

void TerrainPlanner::mavGlobalOriginCallback(const geographic_msgs::GeoPointStampedConstPtr &msg) {
  local_origin_received_ = true;

  double X = static_cast<double>(msg->position.latitude);
  double Y = static_cast<double>(msg->position.longitude);
  double Z = static_cast<double>(msg->position.altitude);
  GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
  double lat, lon, alt;
  earth.Reverse(X, Y, Z, lat, lon, alt);
  // Depending on Gdal versions, lon lat order are reversed
#if GDAL_VERSION_MAJOR > 2
  maneuver_library_->getTerrainMap()->setGlobalOrigin(ESPG::WGS84, Eigen::Vector2d(lat, lon));
#else
  maneuver_library_->getTerrainMap()->setGlobalOrigin(ESPG::WGS84, Eigen::Vector2d(lon, lat));
#endif
  maneuver_library_->getTerrainMap()->setAltitudeOrigin(alt);
}
