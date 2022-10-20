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

#include "terrain_navigation_ros/terrain_planner.h"
#include "terrain_navigation/visualization.h"

#include <grid_map_msgs/GridMap.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/Trajectory.h>
#include <planner_msgs/NavigationStatus.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <GeographicLib/Geocentric.hpp>

TerrainPlanner::TerrainPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private) {
  vehicle_path_pub_ = nh_.advertise<nav_msgs::Path>("vehicle_path", 1);

  grid_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

  posehistory_pub_ = nh_.advertise<nav_msgs::Path>("geometric_controller/path", 10);
  referencehistory_pub_ = nh_.advertise<nav_msgs::Path>("reference/path", 10);
  candidate_manuever_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1, true);
  position_target_pub_ = nh_.advertise<visualization_msgs::Marker>("position_target", 1, true);
  goal_pub_ = nh_.advertise<visualization_msgs::Marker>("goal_marker", 1, true);
  mavstate_sub_ =
      nh_.subscribe("mavros/state", 1, &TerrainPlanner::mavstateCallback, this, ros::TransportHints().tcpNoDelay());
  position_setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1);
  path_target_pub_ = nh_.advertise<mavros_msgs::Trajectory>("mavros/trajectory/generated", 1);
  vehicle_pose_pub_ = nh_.advertise<visualization_msgs::Marker>("vehicle_pose_marker", 1, true);
  planner_status_pub_ = nh_.advertise<planner_msgs::NavigationStatus>("planner_status", 1, true);
  viewpoint_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("viewpoints", 1, true);
  tree_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("tree", 1, true);

  mavpose_sub_ = nh_.subscribe("mavros/local_position/pose", 1, &TerrainPlanner::mavposeCallback, this,
                               ros::TransportHints().tcpNoDelay());
  mavtwist_sub_ = nh_.subscribe("mavros/local_position/velocity_local", 1, &TerrainPlanner::mavtwistCallback, this,
                                ros::TransportHints().tcpNoDelay());
  global_origin_sub_ = nh_.subscribe("mavros/global_position/gp_origin", 1, &TerrainPlanner::mavGlobalOriginCallback,
                                     this, ros::TransportHints().tcpNoDelay());
  image_captured_sub_ = nh_.subscribe("mavros/camera/image_captured", 1, &TerrainPlanner::mavImageCapturedCallback,
                                      this, ros::TransportHints().tcpNoDelay());

  setlocation_serviceserver_ =
      nh_.advertiseService("/terrain_planner/set_location", &TerrainPlanner::setLocationCallback, this);
  setmaxaltitude_serviceserver_ =
      nh_.advertiseService("/terrain_planner/set_max_altitude", &TerrainPlanner::setMaxAltitudeCallback, this);
  setgoal_serviceserver_ = nh_.advertiseService("/terrain_planner/set_goal", &TerrainPlanner::setGoalCallback, this);
  msginterval_serviceclient_ = nh_.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

  nh_private.param<std::string>("terrain_path", map_path_, "resources/cadastre.tif");
  nh_private.param<std::string>("terrain_color_path", map_color_path_, "");
  nh_private.param<std::string>("resource_path", resource_path_, "resources");
  nh_private.param<std::string>("meshresource_path", mesh_resource_path_, "../resources/believer.dae");
  maneuver_library_ = std::make_shared<ManeuverLibrary>();
  maneuver_library_->setPlanningHorizon(4.0);

  primitive_planner_ = std::make_shared<PrimitivePlanner>();
  terrain_map_ = std::make_shared<TerrainMap>();
  viewutility_map_ = std::make_shared<ViewUtilityMap>(terrain_map_->getGridMap());

  maneuver_library_->setTerrainMap(terrain_map_);
  primitive_planner_->setTerrainMap(terrain_map_);

  mcts_planner_ = std::make_shared<MctsPlanner>();
  mcts_planner_->setViewUtilityMap(viewutility_map_);

  global_planner_ = std::make_shared<TerrainOmplRrt>();
  global_planner_->setMap(terrain_map_);

  planner_profiler_ = std::make_shared<Profiler>("planner");
}
TerrainPlanner::~TerrainPlanner() {
  // Destructor
}

void TerrainPlanner::Init() {
  double statusloop_dt_ = 2.0;
  ros::TimerOptions statuslooptimer_options(
      ros::Duration(statusloop_dt_), boost::bind(&TerrainPlanner::statusloopCallback, this, _1), &statusloop_queue_);
  statusloop_timer_ = nh_.createTimer(statuslooptimer_options);  // Define timer for constant loop rate

  statusloop_spinner_.reset(new ros::AsyncSpinner(1, &statusloop_queue_));
  statusloop_spinner_->start();
  double cmdloop_dt_ = 0.1;
  ros::TimerOptions cmdlooptimer_options(ros::Duration(cmdloop_dt_),
                                         boost::bind(&TerrainPlanner::cmdloopCallback, this, _1), &cmdloop_queue_);
  cmdloop_timer_ = nh_.createTimer(cmdlooptimer_options);  // Define timer for constant loop rate

  cmdloop_spinner_.reset(new ros::AsyncSpinner(1, &cmdloop_queue_));
  cmdloop_spinner_->start();
}

void TerrainPlanner::cmdloopCallback(const ros::TimerEvent &event) {
  if (!map_initialized_) return;

  switch (setpoint_mode_) {
    case SETPOINT_MODE::STATE: {
      if (!reference_primitive_.segments.empty()) {
        Eigen::Vector3d reference_position;
        Eigen::Vector3d reference_tangent;
        double reference_curvature{0.0};
        reference_primitive_.getClosestPoint(vehicle_position_, reference_position, reference_tangent,
                                             reference_curvature, 0.1 * 15.0);
        publishPositionSetpoints(reference_position, reference_tangent, reference_curvature);
        /// TODO: Trigger camera when viewpoint reached
        /// This can be done using the mavlink message MAV_CMD_IMAGE_START_CAPTURE
        if (current_state_.mode == "OFFBOARD") {
          publishPositionHistory(referencehistory_pub_, reference_position, referencehistory_vector_);
          tracking_error_ = reference_position - vehicle_position_;
          planner_enabled_ = true;

          // Trigger and keep track of viewpoints
          if ((ros::Time::now() - last_triggered_time_).toSec() > 1.0) {
            const int id = viewpoints_.size() + added_viewpoint_list.size();
            ViewPoint viewpoint(id, vehicle_position_, vehicle_attitude_);
            added_viewpoint_list.push_back(viewpoint);
            last_triggered_time_ = ros::Time::now();
          }

        } else {
          tracking_error_ = Eigen::Vector3d::Zero();
          planner_enabled_ = false;
        }
      }
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
    std::cout << "[TerrainPlanner] Local origin received, loading map" << std::endl;
    map_initialized_ = terrain_map_->Load(map_path_, true, map_color_path_);
    terrain_map_->AddLayerDistanceTransform(50.0, "distance_surface");
    terrain_map_->AddLayerOffset(150.0, "max_elevation");
    if (map_initialized_) {
      std::cout << "[TerrainPlanner]   - Successfully loaded map: " << map_path_ << std::endl;
      viewutility_map_->initializeFromGridmap();
      global_planner_->setBoundsFromMap(terrain_map_->getGridMap());
    } else {
      std::cout << "[TerrainPlanner]   - Failed to load map: " << map_path_ << std::endl;
    }
    return;
  }
  if (!local_origin_received_) {
    std::cout << "Requesting global origin messages" << std::endl;
    mavros_msgs::CommandLong request_global_origin_msg;
    request_global_origin_msg.request.command = 512;
    request_global_origin_msg.request.param1 = 49;
    msginterval_serviceclient_.call(request_global_origin_msg);
    return;
  }

  planner_profiler_->tic();

  // Plan from the end of the current segment
  if (current_state_.mode != "OFFBOARD") {
    reference_primitive_.segments.clear();
  }
  // Only run planner in offboard mode
  /// TODO: Switch to chrono
  plan_time_ = ros::Time::now();
  planner_mode_ = PLANNER_MODE::GLOBAL;
  switch (planner_mode_) {
    case PLANNER_MODE::MCTS: {
      double utility = viewutility_map_->CalculateViewUtility(added_viewpoint_list, true);
      viewpoints_.insert(viewpoints_.end(), added_viewpoint_list.begin(), added_viewpoint_list.end());
      added_viewpoint_list.clear();
      TrajectorySegments candidate_primitive =
          mcts_planner_->solve(vehicle_position_, vehicle_velocity_, vehicle_attitude_, reference_primitive_);
      if (candidate_primitive.valid()) {
        reference_primitive_ = candidate_primitive;
      }
      break;
    }
    case PLANNER_MODE::GLOBAL: {
      /// TODO: Handle start states with loiter circles
      Eigen::Vector3d start_position = vehicle_position_;
      Eigen::Vector3d start_velocity = vehicle_velocity_;
      // Solve planning problem with RRT*
      if (!reference_primitive_.segments.empty()) {
        Trajectory current_segment = reference_primitive_.getCurrentSegment(vehicle_position_);
        start_position = current_segment.states.back().position;
        start_velocity = current_segment.states.back().velocity;

        if ((start_position != previous_start_position_) && !found_solution_) {
          std::cout << "Start position changed! Updating problem" << std::endl;
          problem_updated_ = true;
        }

        /// Only update the problem when the goal is updated
        if (problem_updated_) {
          problem_updated_ = false;
          previous_start_position_ = start_position;
          Eigen::Vector3d goal_velocity(10.0, 0.0, 0.0);
          global_planner_->setupProblem(start_position, start_velocity, goal_pos_, goal_velocity);
        }

        TrajectorySegments planner_solution_path;
        if (!found_solution_) {
          found_solution_ = global_planner_->Solve(1.0, planner_solution_path);

          /// TODO: Improve solution even if a solution have been found
          if (found_solution_) {
            TrajectorySegments updated_segment;
            updated_segment.segments.clear();
            updated_segment.appendSegment(current_segment);
            updated_segment.appendSegment(planner_solution_path);
            Eigen::Vector3d emergency_rates(0.0, 0.0, 0.3);
            double horizon = 2 * M_PI / emergency_rates(2);
            // expandPrimitives(motion_primitive_tree_, emergency_rates, horizon);
            Eigen::Vector3d end_position = planner_solution_path.lastSegment().states.back().position;
            Eigen::Vector3d end_velocity = planner_solution_path.lastSegment().states.back().velocity;

            // Append a loiter at the end of the planned path
            Trajectory loiter_trajectory =
                maneuver_library_->generateArcTrajectory(emergency_rates, horizon, end_position, end_velocity);
            updated_segment.appendSegment(loiter_trajectory);
            reference_primitive_ = updated_segment;
          }
        }
        publishTree(tree_pub_, global_planner_->getPlannerData(), global_planner_->getProblemSetup());

      } else {
        maneuver_library_->generateMotionPrimitives(vehicle_position_, vehicle_velocity_, vehicle_attitude_,
                                                    reference_primitive_);
        maneuver_library_->Solve();
        reference_primitive_ = maneuver_library_->getBestPrimitive();
      }
      break;
    }
    case PLANNER_MODE::EXHAUSTIVE:
      primitive_planner_->setGoalPosition(goal_pos_);
      reference_primitive_ =
          primitive_planner_->solve(vehicle_position_, vehicle_velocity_, vehicle_attitude_, reference_primitive_);
      publishCandidateManeuvers(primitive_planner_->getMotionPrimitives());

      break;
    case PLANNER_MODE::RANDOM:
    default:
      maneuver_library_->generateMotionPrimitives(vehicle_position_, vehicle_velocity_, vehicle_attitude_,
                                                  reference_primitive_);
      /// TODO: Take failsafe action when no valid primitive is found
      std::cout << "[TerrainPlanner] Unable to found a valid motion primitive: using a random primitive" << std::endl;
      reference_primitive_ = maneuver_library_->getRandomPrimitive();
      break;
  }

  double planner_time = planner_profiler_->toc();
  publishTrajectory(reference_primitive_.position());
  MapPublishOnce();
  publishGoal(maneuver_library_->getGoalPosition());

  planner_msgs::NavigationStatus msg;
  msg.header.stamp = ros::Time::now();
  msg.planner_time.data = planner_time;
  msg.tracking_error = toVector3(tracking_error_);
  msg.enabled = planner_enabled_;
  planner_status_pub_.publish(msg);
  publishViewpoints(viewpoints_);
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
  unsigned int posehistory_window_ = 20000;
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
  msg.type_mask = 0.0;
  msg.position.x = position(0);
  msg.position.y = position(1);
  msg.position.z = position(2);
  msg.velocity.x = velocity(0);
  msg.velocity.y = velocity(1);
  msg.velocity.z = velocity(2);
  auto curvature_vector = Eigen::Vector3d(0.0, 0.0, curvature);
  auto projected_velocity = Eigen::Vector3d(velocity(0), velocity(1), 0.0);
  Eigen::Vector3d lateral_acceleration = projected_velocity.squaredNorm() * curvature_vector.cross(projected_velocity);
  msg.acceleration_or_force.x = lateral_acceleration(0);
  msg.acceleration_or_force.y = lateral_acceleration(1);
  msg.acceleration_or_force.z = lateral_acceleration(2);

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
  double yaw = std::atan2(velocity.y(), velocity.x());
  marker.pose.orientation.w = std::cos(0.5 * yaw);
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = std::sin(0.5 * yaw);

  position_target_pub_.publish(marker);
}

void TerrainPlanner::publishGoal(const Eigen::Vector3d &position) {
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.header.frame_id = "map";
  marker.id = 1;
  marker.action = visualization_msgs::Marker::DELETEALL;
  goal_pub_.publish(marker);

  marker.header.stamp = ros::Time::now();
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 20.0;
  marker.scale.y = 20.0;
  marker.scale.z = 20.0;
  marker.color.a = 0.5;  // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.pose.position.x = position(0);
  marker.pose.position.y = position(1);
  marker.pose.position.z = position(2);
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;

  goal_pub_.publish(marker);
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
  marker.mesh_resource = "package://terrain_planner/" + mesh_resource_path_;
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

void TerrainPlanner::publishViewpoints(std::vector<ViewPoint> &viewpoint_vector) {
  visualization_msgs::MarkerArray msg;

  std::vector<visualization_msgs::Marker> marker;
  visualization_msgs::Marker mark;
  mark.action = visualization_msgs::Marker::DELETEALL;
  marker.push_back(mark);
  msg.markers = marker;
  viewpoint_pub_.publish(msg);

  std::vector<visualization_msgs::Marker> viewpoint_marker_vector;
  int i = 0;
  for (auto viewpoint : viewpoint_vector) {
    viewpoint_marker_vector.insert(viewpoint_marker_vector.begin(), Viewpoint2MarkerMsg(i, viewpoint));
    i++;
  }
  msg.markers = viewpoint_marker_vector;
  viewpoint_pub_.publish(msg);
}

void TerrainPlanner::publishTree(const ros::Publisher &pub, std::shared_ptr<ompl::base::PlannerData> planner_data,
                                 std::shared_ptr<ompl::OmplSetup> problem_setup) {
  visualization_msgs::MarkerArray marker_array;
  planner_data->decoupleFromPlanner();

  // allocate variables
  std::vector<unsigned int> edge_list;

  // Create states, a marker and a list to store edges
  ompl::base::ScopedState<fw_planning::spaces::DubinsAirplaneStateSpace> vertex(problem_setup->getSpaceInformation());
  ompl::base::ScopedState<fw_planning::spaces::DubinsAirplaneStateSpace> neighbor_vertex(
      problem_setup->getSpaceInformation());
  size_t marker_idx{0};
  auto dubins_ss = std::make_shared<fw_planning::spaces::DubinsAirplaneStateSpace>();
  for (size_t i = 0; i < planner_data->numVertices(); i++) {
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time().now();
    marker.header.frame_id = "map";
    vertex = planner_data->getVertex(i).getState();
    marker.ns = "vertex";
    marker.id = marker_idx++;
    marker.pose.position.x = vertex[0];
    marker.pose.position.y = vertex[1];
    marker.pose.position.z = vertex[2];
    marker.pose.orientation.w = std::cos(0.5 * vertex[3]);
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = std::sin(0.5 * vertex[3]);
    marker.scale.x = 10.0;
    marker.scale.y = 2.0;
    marker.scale.z = 2.0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = 0.5;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker_array.markers.push_back(marker);

    int num_edges = planner_data->getEdges(i, edge_list);
    if (num_edges > 0) {
      for (unsigned int edge : edge_list) {
        visualization_msgs::Marker edge_marker;
        edge_marker.header.stamp = ros::Time().now();
        edge_marker.header.frame_id = "map";
        edge_marker.id = marker_idx++;
        edge_marker.type = visualization_msgs::Marker::LINE_STRIP;
        edge_marker.ns = "edge";
        neighbor_vertex = planner_data->getVertex(edge).getState();
        // points.push_back(toMsg(Eigen::Vector3d(vertex[0], vertex[1], vertex[2])));
        // points.push_back(toMsg(Eigen::Vector3d(neighbor_vertex[0], neighbor_vertex[1], neighbor_vertex[2])));
        ompl::base::State *state = dubins_ss->allocState();
        ompl::base::State *from = dubins_ss->allocState();
        from->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setX(vertex[0]);
        from->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setY(vertex[1]);
        from->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setZ(vertex[2]);
        from->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setYaw(vertex[3]);

        ompl::base::State *to = dubins_ss->allocState();
        to->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setX(neighbor_vertex[0]);
        to->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setY(neighbor_vertex[1]);
        to->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setZ(neighbor_vertex[2]);
        to->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setYaw(neighbor_vertex[3]);

        std::vector<geometry_msgs::Point> points;
        for (double t = 0.0; t <= 1.0; t += 0.02) {
          dubins_ss->interpolate(from, to, t, state);
          auto interpolated_state =
              Eigen::Vector3d(state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getX(),
                              state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getY(),
                              state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getZ());
          points.push_back(toMsg(interpolated_state));
        }
        points.push_back(toMsg(Eigen::Vector3d(neighbor_vertex[0], neighbor_vertex[1], neighbor_vertex[2])));
        edge_marker.points = points;
        edge_marker.action = visualization_msgs::Marker::ADD;
        edge_marker.pose.orientation.w = 1.0;
        edge_marker.pose.orientation.x = 0.0;
        edge_marker.pose.orientation.y = 0.0;
        edge_marker.pose.orientation.z = 0.0;
        edge_marker.scale.x = 1.0;
        edge_marker.scale.y = 1.0;
        edge_marker.scale.z = 1.0;
        edge_marker.color.a = 0.5;  // Don't forget to set the alpha!
        edge_marker.color.r = 1.0;
        edge_marker.color.g = 1.0;
        edge_marker.color.b = 0.0;
        marker_array.markers.push_back(edge_marker);
      }
    }
  }
  pub.publish(marker_array);
}

void TerrainPlanner::mavGlobalOriginCallback(const geographic_msgs::GeoPointStampedConstPtr &msg) {
  std::cout << "[TerrainPlanner] Received Global Origin from FMU" << std::endl;

  local_origin_received_ = true;

  double X = static_cast<double>(msg->position.latitude);
  double Y = static_cast<double>(msg->position.longitude);
  double Z = static_cast<double>(msg->position.altitude);
  GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
  double lat, lon, alt;
  earth.Reverse(X, Y, Z, lat, lon, alt);

  // Depending on Gdal versions, lon lat order are reversed
#if GDAL_VERSION_MAJOR > 2
  maneuver_library_->getTerrainMap()->setGlobalOrigin(ESPG::WGS84, Eigen::Vector3d(lat, lon, alt));
#else
  maneuver_library_->getTerrainMap()->setGlobalOrigin(ESPG::WGS84, Eigen::Vector3d(lon, lat, alt));
#endif
  maneuver_library_->getTerrainMap()->setAltitudeOrigin(alt);
}

void TerrainPlanner::mavImageCapturedCallback(const mavros_msgs::CameraImageCaptured::ConstPtr &msg) {
  // Publish recorded viewpoints
  /// TODO: Transform image tag into local position
  int id = viewpoints_.size();
  ViewPoint viewpoint(id, vehicle_position_, vehicle_attitude_);
  if (viewutility_map_) viewutility_map_->UpdateUtility(viewpoint);
  viewpoints_.push_back(viewpoint);
  publishViewpoints(viewpoints_);
}

bool TerrainPlanner::setLocationCallback(planner_msgs::SetString::Request &req,
                                         planner_msgs::SetString::Response &res) {
  std::string set_location = req.string;
  bool align_location = req.align;
  std::cout << "[TerrainPlanner] Set Location: " << set_location << std::endl;
  std::cout << "[TerrainPlanner] Set Alignment: " << align_location << std::endl;

  /// TODO: Add location from the new set location service
  map_path_ = resource_path_ + "/" + set_location + ".tif";
  map_color_path_ = resource_path_ + "/" + set_location + "_color.tif";
  bool result = terrain_map_->Load(map_path_, align_location, map_color_path_);
  if (result) {
    global_planner_->setBoundsFromMap(terrain_map_->getGridMap());
    problem_updated_ = true;
    found_solution_ = false;
  }
  res.success = result;
  return true;
}

bool TerrainPlanner::setMaxAltitudeCallback(planner_msgs::SetString::Request &req,
                                            planner_msgs::SetString::Response &res) {
  bool set_max_alitude_constraint = req.align;
  std::cout << "[TerrainPlanner] Max altitude constraint configured: " << set_max_alitude_constraint << std::endl;
  maneuver_library_->setMaxAltitudeConstraint(set_max_alitude_constraint);
  res.success = true;
  return true;
}

bool TerrainPlanner::setGoalCallback(planner_msgs::SetVector3::Request &req, planner_msgs::SetVector3::Response &res) {
  Eigen::Vector3d new_goal = Eigen::Vector3d(req.vector.x, req.vector.y, req.vector.z);
  new_goal = maneuver_library_->setTerrainRelativeGoalPosition(new_goal);
  mcts_planner_->setGoal(new_goal);
  goal_pos_ = maneuver_library_->getGoalPosition();
  problem_updated_ = true;
  found_solution_ = false;

  res.success = true;
  return true;
}
