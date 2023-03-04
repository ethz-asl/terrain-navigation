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
#include "terrain_navigation_ros/geo_conversions.h"

#include <grid_map_msgs/GridMap.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/Trajectory.h>
#include <planner_msgs/NavigationStatus.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

TerrainPlanner::TerrainPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private) {
  vehicle_path_pub_ = nh_.advertise<nav_msgs::Path>("vehicle_path", 1);

  grid_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

  posehistory_pub_ = nh_.advertise<nav_msgs::Path>("geometric_controller/path", 10);
  referencehistory_pub_ = nh_.advertise<nav_msgs::Path>("reference/path", 10);
  position_target_pub_ = nh_.advertise<visualization_msgs::Marker>("position_target", 1, true);
  vehicle_velocity_pub_ = nh_.advertise<visualization_msgs::Marker>("vehicle_velocity", 1, true);
  goal_pub_ = nh_.advertise<visualization_msgs::Marker>("goal_marker", 1, true);
  candidate_goal_pub_ = nh_.advertise<visualization_msgs::Marker>("candidate_goal_marker", 1, true);
  candidate_start_pub_ = nh_.advertise<visualization_msgs::Marker>("candidate_start_marker", 1, true);
  mavstate_sub_ =
      nh_.subscribe("mavros/state", 1, &TerrainPlanner::mavstateCallback, this, ros::TransportHints().tcpNoDelay());
  position_setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1);
  global_position_setpoint_pub_ = nh_.advertise<mavros_msgs::GlobalPositionTarget>("mavros/setpoint_raw/global", 1);
  path_target_pub_ = nh_.advertise<mavros_msgs::Trajectory>("mavros/trajectory/generated", 1);
  vehicle_pose_pub_ = nh_.advertise<visualization_msgs::Marker>("vehicle_pose_marker", 1, true);
  planner_status_pub_ = nh_.advertise<planner_msgs::NavigationStatus>("planner_status", 1, true);
  viewpoint_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("viewpoints", 1, true);
  path_segment_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("path_segments", 1, true);
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
  setstart_serviceserver_ = nh_.advertiseService("/terrain_planner/set_start", &TerrainPlanner::setStartCallback, this);
  setplanning_serviceserver_ =
      nh_.advertiseService("/terrain_planner/trigger_planning", &TerrainPlanner::setPlanningCallback, this);
  updatepath_serviceserver_ = nh_.advertiseService("/terrain_planner/set_path", &TerrainPlanner::setPathCallback, this);
  msginterval_serviceclient_ = nh_.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

  nh_private.param<std::string>("terrain_path", map_path_, "resources/cadastre.tif");
  nh_private.param<std::string>("terrain_color_path", map_color_path_, "");
  nh_private.param<std::string>("resource_path", resource_path_, "resources");
  nh_private.param<std::string>("meshresource_path", mesh_resource_path_, "../resources/believer.dae");
  maneuver_library_ = std::make_shared<ManeuverLibrary>();
  maneuver_library_->setPlanningHorizon(4.0);

  primitive_planner_ = std::make_shared<PrimitivePlanner>();
  terrain_map_ = std::make_shared<TerrainMap>();
  // viewutility_map_ = std::make_shared<ViewUtilityMap>(terrain_map_->getGridMap());

  maneuver_library_->setTerrainMap(terrain_map_);
  primitive_planner_->setTerrainMap(terrain_map_);

  // mcts_planner_ = std::make_shared<MctsPlanner>();
  // mcts_planner_->setViewUtilityMap(viewutility_map_);

  global_planner_ = std::make_shared<TerrainOmplRrt>();
  global_planner_->setMap(terrain_map_);
  global_planner_->setAltitudeLimits(max_elevation_, min_elevation_);

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
                                             reference_curvature, 1.0);
        // Publish global position setpoints in the global frame
        ESPG map_coordinate;
        Eigen::Vector3d map_origin;
        terrain_map_->getGlobalOrigin(map_coordinate, map_origin);
        /// TODO: convert reference position to global
        const Eigen::Vector3d lv03_reference_position = reference_position + map_origin;
        double latitude;
        double longitude;
        double altitude;
        GeoConversions::reverse(lv03_reference_position(0), lv03_reference_position(1), lv03_reference_position(2),
                                latitude, longitude, altitude);
        publishGlobalPositionSetpoints(global_position_setpoint_pub_, latitude, longitude, altitude, reference_tangent,
                                       reference_curvature);
        publishReferenceMarker(position_target_pub_, reference_position, reference_tangent, reference_curvature);
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
  publishVelocityMarker(vehicle_velocity_pub_, vehicle_position_, vehicle_velocity_);
  publishPositionHistory(posehistory_pub_, vehicle_position_, posehistory_vector_);
}

void TerrainPlanner::statusloopCallback(const ros::TimerEvent &event) {
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  if (local_origin_received_ && !map_initialized_) {
    std::cout << "[TerrainPlanner] Local origin received, loading map" << std::endl;
    map_initialized_ = terrain_map_->Load(map_path_, true, map_color_path_);
    terrain_map_->AddLayerDistanceTransform(min_elevation_, "distance_surface");
    terrain_map_->AddLayerDistanceTransform(max_elevation_, "max_elevation");
    double radius = 66.667;
    terrain_map_->AddLayerHorizontalDistanceTransform(radius, "ics_+", "distance_surface");
    terrain_map_->AddLayerHorizontalDistanceTransform(-radius, "ics_-", "max_elevation");
    if (map_initialized_) {
      std::cout << "[TerrainPlanner]   - Successfully loaded map: " << map_path_ << std::endl;
      // viewutility_map_->initializeFromGridmap();
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

  // std::cout << "Terrain altitude: " << terrain_map_->getGridMap().atPosition("elevation", vehicle_position_.head(2))
  // << std::endl; Plan from the end of the current segment
  // Only run planner in offboard mode
  /// TODO: Switch to chrono
  planner_mode_ = PLANNER_MODE::GLOBAL;
  switch (planner_mode_) {
    case PLANNER_MODE::GLOBAL_REPLANNING: {
      if (current_state_.mode != "OFFBOARD") {
        reference_primitive_.segments.clear();
      }
      /// TODO: Handle start states with loiter circles
      Eigen::Vector3d start_position = vehicle_position_;
      Eigen::Vector3d start_velocity = vehicle_velocity_;
      // Solve planning problem with RRT*
      if (!reference_primitive_.segments.empty()) {
        Trajectory current_segment = reference_primitive_.getCurrentSegment(vehicle_position_);
        start_position = current_segment.states.back().position;
        start_velocity = current_segment.states.back().velocity;

        if ((start_position != previous_start_position_)) {
          std::cout << "Start position changed! Updating problem" << std::endl;
          problem_updated_ = true;
        }

        /// Only update the problem when the goal is updated
        bool new_problem = false;
        if (problem_updated_) {
          previous_start_position_ = start_position;
          Eigen::Vector3d goal_velocity(10.0, 0.0, 0.0);
          /// TODO: Get start position and goal velocity
          global_planner_->setupProblem(start_position, start_velocity, goal_pos_);
          problem_updated_ = false;
          new_problem = true;
        }

        TrajectorySegments planner_solution_path;
        bool found_solution = global_planner_->Solve(1.0, planner_solution_path);

        // If a solution is found, check if the new solution is better than the previous solution
        if (found_solution) {
          bool update_solution{false};

          if (new_problem) {
            new_problem = false;  // Since a solution is found, it is not a new problem anymore
            update_solution = planner_solution_path.segments.empty() ? false : true;
            std::cout << "-------------------" << std::endl;
            std::cout << "  - new problem and update solution " << std::endl;
            std::cout << "    - update_solution: " << update_solution << std::endl;
          } else {  /// Check if the found solution is a better solution
            /// Get length of the new planner solution path
            double new_solution_path_length = planner_solution_path.getLength();
            std::cout << "-------------------" << std::endl;
            std::cout << "  - new_solution_path_length: " << new_solution_path_length << std::endl;

            /// Get length of the left solution path of the current path segment
            const int current_idx = reference_primitive_.getCurrentSegmentIndex(vehicle_position_);
            double current_solution_path_length = reference_primitive_.getLength(current_idx + 1);
            double total_solution_path_length = reference_primitive_.getLength(0);
            std::cout << "    - current_solution total path_length: " << total_solution_path_length << std::endl;
            std::cout << "    - current_solution_path_length: " << current_solution_path_length << std::endl;
            /// Compare path length between the two path lengths
            update_solution = bool(new_solution_path_length < current_solution_path_length) &&
                              bool(planner_solution_path.segments.size() > 0);
            std::cout << "  - Found better solution: " << update_solution << std::endl;
          }

          // If a better solution is found, update the path
          if (update_solution) {
            std::cout << "  - Updating solution" << std::endl;
            TrajectorySegments updated_segment;
            updated_segment.segments.clear();
            updated_segment.appendSegment(current_segment);
            updated_segment.appendSegment(planner_solution_path);

            Eigen::Vector3d end_position = planner_solution_path.lastSegment().states.back().position;
            Eigen::Vector3d end_velocity = planner_solution_path.lastSegment().states.back().velocity;
            Eigen::Vector3d radial_vector = (end_position - goal_pos_);
            radial_vector(2) = 0.0;  // Only consider horizontal loiters
            Eigen::Vector3d emergency_rates =
                20.0 * end_velocity.normalized().cross(radial_vector.normalized()) / radial_vector.norm();
            double horizon = 2 * M_PI / std::abs(emergency_rates(2));
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
    case PLANNER_MODE::GLOBAL: {
      // Solve planning problem with RRT*
      /// TODO: Plan on execution
      bool run_planner = true;
      double time_spent_planning = ros::Duration(ros::Time::now() - plan_time_).toSec();
      if (time_spent_planning < planner_time_budget_) {
        bool found_solution = global_planner_->Solve(1.0, candidate_primitive_);
        publishTree(tree_pub_, global_planner_->getPlannerData(), global_planner_->getProblemSetup());
      }
      publishPathSegments(path_segment_pub_, candidate_primitive_);
      break;
    }
    case PLANNER_MODE::EXHAUSTIVE:
      primitive_planner_->setGoalPosition(goal_pos_);
      primitive_planner_->setup(vehicle_position_, vehicle_velocity_, vehicle_attitude_, reference_primitive_);
      reference_primitive_ = primitive_planner_->solve();
      publishCandidateManeuvers(tree_pub_, primitive_planner_->getMotionPrimitives());

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
  publishPathSegments(path_segment_pub_, reference_primitive_);
  MapPublishOnce();
  // publishGoal(goal_pub_, goal_pos_, 66.67, Eigen::Vector3d(0.0, 1.0, 0.0));

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
  const Eigen::Vector3d local_vehicle_position = toEigen(msg.pose.position);

  if (enu_.has_value()) {  // Only update position if the transforms have been initialized
    Eigen::Vector3d wgs84_vehicle_position = toEigen(msg.pose.position);
    // Depending on Gdal versions, lon lat order are reversed
    enu_.value().Reverse(local_vehicle_position.x(), local_vehicle_position.y(), local_vehicle_position.z(),
                         wgs84_vehicle_position.x(), wgs84_vehicle_position.y(), wgs84_vehicle_position.z());

    ESPG map_coordinate;
    Eigen::Vector3d map_origin;
    terrain_map_->getGlobalOrigin(map_coordinate, map_origin);

    if (map_coordinate == ESPG::WGS84) {
      GeoConversions::forward(map_origin(0), map_origin(1), map_origin(2), map_origin.x(), map_origin.y(),
                              map_origin.z());
    }

    Eigen::Vector3d transformed_coordinates;
    // LV03 / WGS84 ellipsoid
    GeoConversions::forward(wgs84_vehicle_position(0), wgs84_vehicle_position(1), wgs84_vehicle_position(2),
                            transformed_coordinates.x(), transformed_coordinates.y(), transformed_coordinates.z());
    vehicle_position_ = transformed_coordinates - map_origin;

    vehicle_attitude_(0) = msg.pose.orientation.w;
    vehicle_attitude_(1) = msg.pose.orientation.x;
    vehicle_attitude_(2) = msg.pose.orientation.y;
    vehicle_attitude_(3) = msg.pose.orientation.z;
  }
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

void TerrainPlanner::publishGlobalPositionSetpoints(const ros::Publisher &pub, const double latitude,
                                                    const double longitude, const double altitude,
                                                    const Eigen::Vector3d &velocity, const double curvature) {
  using namespace mavros_msgs;
  // Publishes position setpoints sequentially as trajectory setpoints
  mavros_msgs::GlobalPositionTarget msg;
  msg.header.stamp = ros::Time::now();
  msg.coordinate_frame = GlobalPositionTarget::FRAME_GLOBAL_REL_ALT;
  msg.type_mask = 0.0;
  msg.latitude = latitude;
  msg.longitude = longitude;
  msg.altitude = altitude - local_origin_altitude_;
  msg.velocity.x = velocity(0);
  msg.velocity.y = velocity(1);
  msg.velocity.z = velocity(2);
  auto curvature_vector = Eigen::Vector3d(0.0, 0.0, -curvature);
  auto projected_velocity = Eigen::Vector3d(velocity(0), velocity(1), 0.0);
  Eigen::Vector3d lateral_acceleration = projected_velocity.squaredNorm() * curvature_vector.cross(projected_velocity);
  msg.acceleration_or_force.x = lateral_acceleration(0);
  msg.acceleration_or_force.y = lateral_acceleration(1);
  msg.acceleration_or_force.z = lateral_acceleration(2);

  pub.publish(msg);
}

void TerrainPlanner::publishPositionSetpoints(const ros::Publisher &pub, const Eigen::Vector3d &position,
                                              const Eigen::Vector3d &velocity, const double curvature) {
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
  auto curvature_vector = Eigen::Vector3d(0.0, 0.0, -curvature);
  auto projected_velocity = Eigen::Vector3d(velocity(0), velocity(1), 0.0);
  Eigen::Vector3d lateral_acceleration = projected_velocity.squaredNorm() * curvature_vector.cross(projected_velocity);
  msg.acceleration_or_force.x = lateral_acceleration(0);
  msg.acceleration_or_force.y = lateral_acceleration(1);
  msg.acceleration_or_force.z = lateral_acceleration(2);

  pub.publish(msg);
}

void TerrainPlanner::publishVelocityMarker(const ros::Publisher &pub, const Eigen::Vector3d &position,
                                           const Eigen::Vector3d &velocity) {
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::ARROW;
  marker.header.frame_id = "map";
  marker.id = 0;
  marker.action = visualization_msgs::Marker::DELETEALL;
  position_target_pub_.publish(marker);

  marker.header.stamp = ros::Time::now();
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = velocity.norm();
  marker.scale.y = 2.0;
  marker.scale.z = 2.0;
  marker.color.a = 0.5;  // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.pose.position.x = position(0);
  marker.pose.position.y = position(1);
  marker.pose.position.z = position(2);
  double yaw = std::atan2(velocity.y(), velocity.x());
  double pitch = std::atan2(velocity.z(), velocity.x());
  marker.pose.orientation.w = std::cos(0.5 * yaw);
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = std::sin(0.5 * yaw);

  pub.publish(marker);
}

void TerrainPlanner::publishReferenceMarker(const ros::Publisher &pub, const Eigen::Vector3d &position,
                                            const Eigen::Vector3d &velocity, const double curvature) {
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

  pub.publish(marker);
}

void TerrainPlanner::publishPathSegments(ros::Publisher &pub, TrajectorySegments &trajectory) {
  visualization_msgs::MarkerArray msg;

  std::vector<visualization_msgs::Marker> marker;
  visualization_msgs::Marker mark;
  mark.action = visualization_msgs::Marker::DELETEALL;
  marker.push_back(mark);
  msg.markers = marker;
  pub.publish(msg);

  std::vector<visualization_msgs::Marker> segment_markers;
  int i = 0;
  for (auto &segment : trajectory.segments) {
    Eigen::Vector3d color = Eigen::Vector3d(1.0, 0.0, 0.0);
    if (segment.curvature > 0.0) {  // Green is DUBINS_LEFT
      color = Eigen::Vector3d(0.0, 1.0, 0.0);
    } else if (segment.curvature < 0.0) {  // Blue is DUBINS_RIGHT
      color = Eigen::Vector3d(0.0, 0.0, 1.0);
    }
    segment_markers.insert(segment_markers.begin(), trajectory2MarkerMsg(segment, i++, color));
    segment_markers.insert(segment_markers.begin(), point2MarkerMsg(segment.position().front(), i++, color));
    segment_markers.insert(segment_markers.begin(), point2MarkerMsg(segment.position().back(), i++, color));
  }
  msg.markers = segment_markers;
  pub.publish(msg);
}

void TerrainPlanner::publishGoal(const ros::Publisher &pub, const Eigen::Vector3d &position, const double radius,
                                 Eigen::Vector3d color) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "goal";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  std::vector<geometry_msgs::Point> points;
  /// TODO: Generate circular path
  double delta_theta = 0.05 * 2 * M_PI;
  for (double theta = 0.0; theta < 2 * M_PI + delta_theta; theta += delta_theta) {
    geometry_msgs::Point point;
    point.x = position(0) + radius * std::cos(theta);
    point.y = position(1) + radius * std::sin(theta);
    point.z = position(2);
    points.push_back(point);
  }
  marker.points = points;
  marker.scale.x = 5.0;
  marker.scale.y = 5.0;
  marker.scale.z = 5.0;
  marker.color.a = 0.5;  // Don't forget to set the alpha!
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.color.r = color(0);
  marker.color.g = color(1);
  marker.color.b = color(2);

  pub.publish(marker);
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

void TerrainPlanner::mavGlobalOriginCallback(const geographic_msgs::GeoPointStampedConstPtr &msg) {
  std::cout << "[TerrainPlanner] Received Global Origin from FMU" << std::endl;

  local_origin_received_ = true;

  double X = static_cast<double>(msg->position.latitude);
  double Y = static_cast<double>(msg->position.longitude);
  double Z = static_cast<double>(msg->position.altitude);
  GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
  double lat, lon, alt;
  earth.Reverse(X, Y, Z, lat, lon, alt);
  enu_.emplace(lat, lon, alt, GeographicLib::Geocentric::WGS84());
  local_origin_altitude_ = alt;
  local_origin_latitude_ = lat;
  local_origin_longitude_ = lon;
}

void TerrainPlanner::mavImageCapturedCallback(const mavros_msgs::CameraImageCaptured::ConstPtr &msg) {
  // Publish recorded viewpoints
  /// TODO: Transform image tag into local position
  int id = viewpoints_.size();
  ViewPoint viewpoint(id, vehicle_position_, vehicle_attitude_);
  // if (viewutility_map_) viewutility_map_->UpdateUtility(viewpoint);
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
  terrain_map_->AddLayerDistanceTransform(min_elevation_, "distance_surface");
  terrain_map_->AddLayerDistanceTransform(max_elevation_, "max_elevation");
  double radius = 66.667;
  terrain_map_->AddLayerHorizontalDistanceTransform(radius, "ics_+", "distance_surface");
  terrain_map_->AddLayerHorizontalDistanceTransform(-radius, "ics_-", "max_elevation");
  global_planner_->setBoundsFromMap(terrain_map_->getGridMap());

  if (!align_location) {
    // Depending on Gdal versions, lon lat order are reversed
    Eigen::Vector3d lv03_local_origin;
    GeoConversions::forward(local_origin_latitude_, local_origin_longitude_, local_origin_altitude_,
                            lv03_local_origin.x(), lv03_local_origin.y(), lv03_local_origin.z());
    double map_origin_altitude = local_origin_altitude_;
    if (terrain_map_->getGridMap().isInside(Eigen::Vector2d(0.0, 0.0))) {
      double terrain_altitude = terrain_map_->getGridMap().atPosition("elevation", Eigen::Vector2d(0.0, 0.0));
      lv03_local_origin(2) = lv03_local_origin(2) - terrain_altitude;
    }
    terrain_map_->setGlobalOrigin(ESPG::CH1903_LV03, lv03_local_origin);
  }
  if (result) {
    global_planner_->setBoundsFromMap(terrain_map_->getGridMap());
    problem_updated_ = true;
    posehistory_vector_.clear();
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
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  Eigen::Vector3d candidate_goal = Eigen::Vector3d(req.vector.x, req.vector.y, req.vector.z);
  Eigen::Vector3d new_goal;
  std::cout << "[TerrainPlanner] Candidate goal: " << candidate_goal.transpose() << std::endl;
  bool is_goal_safe = validatePosition(terrain_map_->getGridMap(), candidate_goal, new_goal);
  if (is_goal_safe) {
    goal_pos_ = new_goal;
    // mcts_planner_->setGoal(new_goal);
    // problem_updated_ = true;
    res.success = true;
    publishGoal(candidate_goal_pub_, new_goal, 66.67, Eigen::Vector3d(0.0, 1.0, 0.0));
    return true;
  } else {
    res.success = false;
    publishGoal(candidate_goal_pub_, new_goal, 66.67, Eigen::Vector3d(1.0, 0.0, 0.0));
    return false;
  }
}

bool TerrainPlanner::setStartCallback(planner_msgs::SetVector3::Request &req, planner_msgs::SetVector3::Response &res) {
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  Eigen::Vector3d candidate_start = Eigen::Vector3d(req.vector.x, req.vector.y, req.vector.z);
  Eigen::Vector3d new_start;
  std::cout << "[TerrainPlanner] Candidate start: " << candidate_start.transpose() << std::endl;
  bool is_safe = validatePosition(terrain_map_->getGridMap(), candidate_start, new_start);
  if (is_safe) {
    start_pos_ = new_start;
    res.success = true;
    publishGoal(candidate_start_pub_, start_pos_, 66.67, Eigen::Vector3d(0.0, 1.0, 0.0));
    return true;
  } else {
    res.success = false;
    publishGoal(candidate_start_pub_, start_pos_, 66.67, Eigen::Vector3d(1.0, 0.0, 0.0));
    return false;
  }
}

bool TerrainPlanner::setPlanningCallback(planner_msgs::SetVector3::Request &req,
                                         planner_msgs::SetVector3::Response &res) {
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  planner_time_budget_ = req.vector.z;
  std::cout << "[TerrainPlanner] Planning budget: " << planner_time_budget_ << std::endl;
  problem_updated_ = true;
  plan_time_ = ros::Time::now();
  global_planner_->setupProblem(start_pos_, goal_pos_);
  res.success = true;
  return true;
}

bool TerrainPlanner::setPathCallback(planner_msgs::SetVector3::Request &req, planner_msgs::SetVector3::Response &res) {
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  planner_time_budget_ = req.vector.z;
  std::cout << "[TerrainPlanner] Planning budget: " << planner_time_budget_ << std::endl;
  problem_updated_ = true;
  plan_time_ = ros::Time::now();
  reference_primitive_ = candidate_primitive_;
  res.success = true;
  return true;
}
