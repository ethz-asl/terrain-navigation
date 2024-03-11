/****************************************************************************
 *
 *   Copyright (c) 2023 Jaeyoung Lim, Autonomous Systems Lab,
 *  ETH Zürich. All rights reserved.
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,R
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
#include "terrain_navigation_ros/geo_conversions.h"
#include "terrain_navigation_ros/visualization.h"

#include <grid_map_msgs/GridMap.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/Trajectory.h>
#include <planner_msgs/NavigationStatus.h>
#include <planner_msgs/Path.h>
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
  position_target_pub_ = nh_.advertise<visualization_msgs::Marker>("position_target", 1);
  vehicle_velocity_pub_ = nh_.advertise<visualization_msgs::Marker>("vehicle_velocity", 1);
  goal_pub_ = nh_.advertise<visualization_msgs::Marker>("goal_marker", 1);
  rallypoint_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("rallypoints_marker", 1);
  candidate_goal_pub_ = nh_.advertise<visualization_msgs::Marker>("candidate_goal_marker", 1);
  candidate_start_pub_ = nh_.advertise<visualization_msgs::Marker>("candidate_start_marker", 1);
  mavstate_sub_ =
      nh_.subscribe("mavros/state", 1, &TerrainPlanner::mavstateCallback, this, ros::TransportHints().tcpNoDelay());
  mavmission_sub_ = nh_.subscribe("mavros/mission/waypoints", 1, &TerrainPlanner::mavMissionCallback, this,
                                  ros::TransportHints().tcpNoDelay());
  global_position_setpoint_pub_ = nh_.advertise<mavros_msgs::GlobalPositionTarget>("mavros/setpoint_raw/global", 1);
  path_target_pub_ = nh_.advertise<mavros_msgs::Trajectory>("mavros/trajectory/generated", 1);
  vehicle_pose_pub_ = nh_.advertise<visualization_msgs::Marker>("vehicle_pose_marker", 1);
  camera_pose_pub_ = nh_.advertise<visualization_msgs::Marker>("camera_pose_marker", 1);
  planner_status_pub_ = nh_.advertise<planner_msgs::NavigationStatus>("planner_status", 1);
  path_segment_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("path_segments", 1);
  tree_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("tree", 1);
  reference_path_pub_ = nh_.advertise<planner_msgs::Path>("reference_path", 1);

  mavlocalpose_sub_ = nh_.subscribe("mavros/local_position/pose", 1, &TerrainPlanner::mavLocalPoseCallback, this,
                                    ros::TransportHints().tcpNoDelay());
  mavglobalpose_sub_ = nh_.subscribe("mavros/global_position/global", 1, &TerrainPlanner::mavGlobalPoseCallback, this,
                                     ros::TransportHints().tcpNoDelay());
  mavtwist_sub_ = nh_.subscribe("mavros/local_position/velocity_local", 1, &TerrainPlanner::mavtwistCallback, this,
                                ros::TransportHints().tcpNoDelay());

  setlocation_serviceserver_ =
      nh_.advertiseService("/terrain_planner/set_location", &TerrainPlanner::setLocationCallback, this);
  setmaxaltitude_serviceserver_ =
      nh_.advertiseService("/terrain_planner/set_max_altitude", &TerrainPlanner::setMaxAltitudeCallback, this);
  setgoal_serviceserver_ = nh_.advertiseService("/terrain_planner/set_goal", &TerrainPlanner::setGoalCallback, this);
  setstart_serviceserver_ = nh_.advertiseService("/terrain_planner/set_start", &TerrainPlanner::setStartCallback, this);
  setcurrentsegment_serviceserver_ =
      nh_.advertiseService("/terrain_planner/set_current_segment", &TerrainPlanner::setCurrentSegmentCallback, this);
  setstartloiter_serviceserver_ =
      nh_.advertiseService("/terrain_planner/set_start_loiter", &TerrainPlanner::setStartLoiterCallback, this);
  setplannerstate_service_server_ =
      nh_.advertiseService("/terrain_planner/set_planner_state", &TerrainPlanner::setPlannerStateCallback, this);
  setplanning_serviceserver_ =
      nh_.advertiseService("/terrain_planner/trigger_planning", &TerrainPlanner::setPlanningCallback, this);
  updatepath_serviceserver_ = nh_.advertiseService("/terrain_planner/set_path", &TerrainPlanner::setPathCallback, this);

  std::string avalanche_map_path;
  nh_private.param<std::string>("terrain_path", map_path_, "resources/cadastre.tif");
  nh_private.param<std::string>("terrain_color_path", map_color_path_, "");
  nh_private.param<std::string>("resource_path", resource_path_, "resources");
  nh_private.param<std::string>("meshresource_path", mesh_resource_path_, "../resources/believer.dae");
  nh_private.param<std::string>("avalanche_map_path", avalanche_map_path, "../data/believer.dae");
  nh_private.param<double>("minimum_turn_radius", goal_radius_, 66.67);

  terrain_map_ = std::make_shared<TerrainMap>();

  // Initialize Dubins state space
  dubins_state_space_ = std::make_shared<fw_planning::spaces::DubinsAirplaneStateSpace>(goal_radius_);
  global_planner_ = std::make_shared<TerrainOmplRrt>(ompl::base::StateSpacePtr(dubins_state_space_));
  global_planner_->setMap(terrain_map_);
  global_planner_->setAltitudeLimits(max_elevation_, min_elevation_);

  // f = boost::bind(&TerrainPlanner::dynamicReconfigureCallback, this, _1, _2);
  // server.setCallback(f);

  planner_profiler_ = std::make_shared<Profiler>("planner");
}
TerrainPlanner::~TerrainPlanner() {
  // Destructor
}

void TerrainPlanner::Init() {
  double plannerloop_dt_ = 2.0;
  ros::TimerOptions plannerlooptimer_options(
      ros::Duration(plannerloop_dt_), boost::bind(&TerrainPlanner::plannerloopCallback, this, _1), &plannerloop_queue_);
  plannerloop_timer_ = nh_.createTimer(plannerlooptimer_options);  // Define timer for constant loop rate

  plannerloop_spinner_.reset(new ros::AsyncSpinner(1, &plannerloop_queue_));
  plannerloop_spinner_->start();

  double statusloop_dt_ = 0.5;
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

  if (!reference_primitive_.segments.empty()) {
    Eigen::Vector3d reference_position;
    Eigen::Vector3d reference_tangent;
    double reference_curvature{0.0};
    auto current_segment = reference_primitive_.getCurrentSegment(vehicle_position_);
    double path_progress = current_segment.getClosestPoint(vehicle_position_, reference_position, reference_tangent,
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
    publishReferenceMarker(position_target_pub_, reference_position, reference_tangent, reference_curvature);

    // Run additional altitude control
    double altitude_correction = K_z_ * (vehicle_position_(2) - reference_position(2));
    double climb_rate = cruise_speed_ * std::sin(current_segment.flightpath_angle);
    Eigen::Vector3d velocity_reference = reference_tangent;

    velocity_reference(2) =
        std::min(std::max(altitude_correction - climb_rate, -max_climb_rate_control_), max_climb_rate_control_);

    /// Blend curvature with next segment
    double curvature_reference = reference_curvature;
    int current_segment_idx = reference_primitive_.getCurrentSegmentIndex(vehicle_position_);
    bool is_last_segment = bool(current_segment_idx >= (reference_primitive_.segments.size() - 1));
    if (!is_last_segment) {
      /// Get next segment curvature
      double next_segment_curvature = reference_primitive_.segments[current_segment_idx + 1].curvature;

      /// Blend current curvature with next curvature when close to the end
      double segment_length = current_segment.getLength();
      double cut_off_distance = 10.0;
      double portion = std::min(
          1.0, std::max((path_progress * segment_length - segment_length + cut_off_distance) / cut_off_distance, 0.0));
      curvature_reference = (1 - portion) * reference_curvature + portion * next_segment_curvature;
    }

    publishGlobalPositionSetpoints(global_position_setpoint_pub_, latitude, longitude, altitude, velocity_reference,
                                   curvature_reference);

    /// TODO: Switch mode to planner engaged
    if (current_state_.mode == "OFFBOARD") {
      publishPositionHistory(referencehistory_pub_, reference_position, referencehistory_vector_);
      tracking_error_ = reference_position - vehicle_position_;
      planner_enabled_ = true;
    } else {
      tracking_error_ = Eigen::Vector3d::Zero();
      planner_enabled_ = false;
    }
  }

  planner_msgs::NavigationStatus msg;
  msg.header.stamp = ros::Time::now();
  // msg.planner_time.data = planner_time;
  msg.state = static_cast<uint8_t>(planner_state_);
  msg.tracking_error = toVector3(tracking_error_);
  msg.enabled = planner_enabled_;
  // msg.reference_position = toVector3(reference_position);
  msg.vehicle_position = toVector3(vehicle_position_);
  planner_status_pub_.publish(msg);

  publishVehiclePose(vehicle_pose_pub_, vehicle_position_, vehicle_attitude_, mesh_resource_path_);
  publishVelocityMarker(vehicle_velocity_pub_, vehicle_position_, vehicle_velocity_);
  publishPositionHistory(posehistory_pub_, vehicle_position_, posehistory_vector_);
}

Eigen::Vector4d TerrainPlanner::rpy2quaternion(double roll, double pitch, double yaw) {
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

void TerrainPlanner::statusloopCallback(const ros::TimerEvent &event) {
  // Check if we want to update the planner state if query state and current state is different
  planner_state_ = finiteStateMachine(planner_state_, query_planner_state_);
  if (query_planner_state_ != planner_state_) {  // Query has been rejected, reset
    query_planner_state_ = planner_state_;
  }
  // printPlannerState(planner_state_);
}

void TerrainPlanner::plannerloopCallback(const ros::TimerEvent &event) {
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  if (!map_initialized_) {
    std::cout << "[TerrainPlanner] Local origin received, loading map" << std::endl;
    map_initialized_ = terrain_map_->Load(map_path_, map_color_path_);
    terrain_map_->AddLayerDistanceTransform(min_elevation_, "distance_surface");
    terrain_map_->AddLayerDistanceTransform(max_elevation_, "max_elevation");
    terrain_map_->AddLayerHorizontalDistanceTransform(goal_radius_, "ics_+", "distance_surface");
    terrain_map_->AddLayerHorizontalDistanceTransform(-goal_radius_, "ics_-", "max_elevation");
    terrain_map_->addLayerSafety("safety", "ics_+", "ics_-");

    ESPG map_coordinate;
    Eigen::Vector3d map_origin;
    terrain_map_->getGlobalOrigin(map_coordinate, map_origin);

    if (map_initialized_) {
      std::cout << "[TerrainPlanner]   - Successfully loaded map: " << map_path_ << std::endl;
      MapPublishOnce(grid_map_pub_, terrain_map_->getGridMap());
      global_planner_->setBoundsFromMap(terrain_map_->getGridMap());
      global_planner_->setupProblem(start_pos_, goal_pos_, start_loiter_radius_);
    } else {
      std::cout << "[TerrainPlanner]   - Failed to load map: " << map_path_ << std::endl;
    }
    return;
  }

  // planner_profiler_->tic();

  switch (planner_mode_) {
    case PLANNER_MODE::GLOBAL: {
      // Solve planning problem with RRT*
      double time_spent_planning = ros::Duration(ros::Time::now() - plan_time_).toSec();
      if (time_spent_planning < planner_time_budget_) {
        bool found_solution = global_planner_->Solve(1.0, candidate_primitive_);
        publishTree(tree_pub_, global_planner_->getPlannerData(), global_planner_->getProblemSetup());
      } else {
        publishPathSegments(path_segment_pub_, candidate_primitive_);
      }
      break;
    }
    case PLANNER_MODE::EMERGENCY_ABORT: {
      // Solve planning problem with RRT*
      if (!reference_primitive_.segments.empty()) {
        PathSegment current_segment = reference_primitive_.getCurrentSegment(vehicle_position_);
        Eigen::Vector3d start_position = current_segment.states.back().position;
        Eigen::Vector3d start_velocity = current_segment.states.back().velocity;

        if ((start_position != previous_start_position_ && !found_solution_)) {
          std::cout << "Start position changed! Updating problem" << std::endl;
          problem_updated_ = true;
        }

        /// Only update the problem when the goal is updated
        if (problem_updated_) {
          /// Generate candidate rally points
          const int num_rally_points = 3;
          rally_points.clear();
          for (int i = 0; i < num_rally_points; i++) {
            bool sample_is_valid = false;
            while (!sample_is_valid) {
              Eigen::Vector3d random_sample;
              random_sample(0) = getRandom(-200.0, 200.0);
              random_sample(1) = getRandom(-200.0, 200.0);
              Eigen::Vector3d candidate_loiter_position = start_position + random_sample;
              Eigen::Vector3d new_loiter_position;
              sample_is_valid =
                  validatePosition(terrain_map_->getGridMap(), candidate_loiter_position, new_loiter_position);
              if (sample_is_valid) {
                rally_points.push_back(new_loiter_position);
              }
            }
          }

          global_planner_->setupProblem(start_position, start_velocity, rally_points);
          /// Publish Rally points
          publishRallyPoints(rallypoint_pub_, rally_points, 66.67, Eigen::Vector3d(1.0, 1.0, 0.0));
          previous_start_position_ = start_position;
        }

        Path planner_solution_path;
        bool found_solution = global_planner_->Solve(0.5, planner_solution_path);
        found_solution_ = found_solution;

        // If a solution is found, check if the new solution is better than the previous solution
        if (found_solution_ && problem_updated_) {
          problem_updated_ = false;

          bool update_solution{false};
          update_solution = planner_solution_path.segments.empty() ? false : true;

          // If a better solution is found, update the path
          if (update_solution) {
            std::cout << "  - Updating solution" << std::endl;
            Path updated_segment;
            updated_segment.segments.clear();
            updated_segment.appendSegment(current_segment);
            updated_segment.appendSegment(planner_solution_path);

            Eigen::Vector3d end_position = planner_solution_path.lastSegment().states.back().position;
            Eigen::Vector3d end_velocity = planner_solution_path.lastSegment().states.back().velocity;
            /// TODO: Figure out which rally point the planner is using
            double min_distance_error = std::numeric_limits<double>::infinity();
            int min_distance_index = -1;
            for (int idx = 0; idx < rally_points.size(); idx++) {
              double radial_error =
                  std::abs((end_position - rally_points[idx]).norm() - dubins_state_space_->getMinTurningRadius());
              if (radial_error < min_distance_error) {
                min_distance_index = idx;
                min_distance_error = radial_error;
              }
            }
            Eigen::Vector3d radial_vector = (end_position - rally_points[min_distance_index]);
            radial_vector(2) = 0.0;  // Only consider horizontal loiters
            Eigen::Vector3d emergency_rates =
                20.0 * end_velocity.normalized().cross(radial_vector.normalized()) / radial_vector.norm();
            double horizon = 2 * M_PI / std::abs(emergency_rates(2));
            // Append a loiter at the end of the planned path
            PathSegment loiter_trajectory;
            generateCircle(end_position, end_velocity, rally_points[min_distance_index], loiter_trajectory);
            updated_segment.appendSegment(loiter_trajectory);
            reference_primitive_ = updated_segment;
            candidate_primitive_ = updated_segment;
          }
        }
        publishTree(tree_pub_, global_planner_->getPlannerData(), global_planner_->getProblemSetup());
      }
      publishPathSegments(path_segment_pub_, candidate_primitive_);
      break;
    }
    case PLANNER_MODE::RETURN: {
      // Solve planning problem with RRT*
      if (!reference_primitive_.segments.empty()) {
        PathSegment current_segment = reference_primitive_.getCurrentSegment(vehicle_position_);
        Eigen::Vector3d start_position = current_segment.states.back().position;
        Eigen::Vector3d start_velocity = current_segment.states.back().velocity;

        if ((start_position != previous_return_start_position_ && !found_return_solution_)) {
          std::cout << "Start position changed! Updating problem" << std::endl;
          problem_updated_ = true;
        }

        /// Only update the problem when the goal is updated
        if (problem_updated_) {
          global_planner_->setupProblem(start_position, start_velocity, home_position_, home_position_radius_);
          previous_return_start_position_ = start_position;
        }

        Path planner_solution_path;
        bool found_solution = global_planner_->Solve(0.5, planner_solution_path);
        found_return_solution_ = found_solution;

        // If a solution is found, check if the new solution is better than the previous solution
        if (found_return_solution_ && problem_updated_) {
          problem_updated_ = false;

          bool update_solution{false};
          update_solution = planner_solution_path.segments.empty() ? false : true;

          // If a better solution is found, update the path
          if (update_solution) {
            std::cout << "  - Updating solution" << std::endl;
            Path updated_segment;
            updated_segment.segments.clear();
            updated_segment.appendSegment(current_segment);
            updated_segment.appendSegment(planner_solution_path);

            Eigen::Vector3d end_position = planner_solution_path.lastSegment().states.back().position;
            Eigen::Vector3d end_velocity = planner_solution_path.lastSegment().states.back().velocity;

            Eigen::Vector3d radial_vector = (end_position - home_position_);
            radial_vector(2) = 0.0;  // Only consider horizontal loiters
            Eigen::Vector3d emergency_rates =
                20.0 * end_velocity.normalized().cross(radial_vector.normalized()) / radial_vector.norm();
            double horizon = 2 * M_PI / std::abs(emergency_rates(2));
            // Append a loiter at the end of the planned path
            PathSegment loiter_trajectory;
            generateCircle(end_position, end_velocity, home_position_, loiter_trajectory);
            updated_segment.appendSegment(loiter_trajectory);
            reference_primitive_ = updated_segment;
            candidate_primitive_ = updated_segment;
          }
        }
        publishTree(tree_pub_, global_planner_->getPlannerData(), global_planner_->getProblemSetup());
      }
      publishPathSegments(path_segment_pub_, candidate_primitive_);
      break;
    }
  }

  // double planner_time = planner_profiler_->toc();
  publishTrajectory(reference_primitive_.position());
  publishPath(reference_path_pub_, reference_primitive_);
  // publishGoal(goal_pub_, goal_pos_, 66.67, Eigen::Vector3d(0.0, 1.0, 0.0));
}

PLANNER_STATE TerrainPlanner::finiteStateMachine(const PLANNER_STATE current_state, const PLANNER_STATE query_state) {
  PLANNER_STATE next_state;
  next_state = current_state;
  switch (current_state) {
    case PLANNER_STATE::NAVIGATE: {
      // Switch to Hold when segment has been completed
      int current_segment_idx = reference_primitive_.getCurrentSegmentIndex(vehicle_position_);
      bool is_last_segment = bool(current_segment_idx >= (reference_primitive_.segments.size() - 1));
      if (is_last_segment) {
        /// TODO: Clear candidate primitive
        candidate_primitive_.segments.clear();
        next_state = PLANNER_STATE::HOLD;
      }

      // Stay in hold mode if the current segment is periodic, Otherwise switch to abort
      if (query_state == PLANNER_STATE::ABORT) {
        if (!reference_primitive_.getCurrentSegment(vehicle_position_).is_periodic) {
          next_state = PLANNER_STATE::ABORT;
          planner_mode_ = PLANNER_MODE::EMERGENCY_ABORT;
        } else {
          /// TODO: Get rid of next segments and only keep current segment
        }
      } else if (query_state == PLANNER_STATE::RETURN) {
        next_state = PLANNER_STATE::RETURN;
        planner_mode_ = PLANNER_MODE::RETURN;
      }
      break;
    }
    case PLANNER_STATE::ROLLOUT: {
      /// TODO: Get rollout primitive from active mapper
      // if (candidate_primitive_.valid()) {
      // Update reference if candidate is valid
      // reference_primitive_ = candidate_primitive_;
      // std::cout << "Candidate primitive is valid" << std::endl;
      // }
      reference_primitive_ = rollout_primitive_;
      /// TODO: Add self termination
      if (query_state == PLANNER_STATE::ABORT) {
        if (reference_primitive_.getCurrentSegment(vehicle_position_).is_periodic) {
          next_state = PLANNER_STATE::HOLD;
          planner_mode_ = PLANNER_MODE::GLOBAL;
        } else {
          next_state = query_state;
          planner_mode_ = PLANNER_MODE::EMERGENCY_ABORT;
        }
      }
      break;
    }
    case PLANNER_STATE::HOLD: {
      switch (query_state) {
        case PLANNER_STATE::NAVIGATE: {
          // Check if the candidate primitive is not empty
          if (!candidate_primitive_.segments.empty()) {
            // Add initial loiter
            Eigen::Vector3d start_position = candidate_primitive_.firstSegment().states.front().position;
            Eigen::Vector3d start_velocity = candidate_primitive_.firstSegment().states.front().velocity;
            PathSegment start_loiter;
            generateCircle(start_position, start_velocity, start_pos_, start_loiter);
            candidate_primitive_.prependSegment(start_loiter);

            // Add terminal loiter
            Eigen::Vector3d end_position = candidate_primitive_.lastSegment().states.back().position;
            Eigen::Vector3d end_velocity = candidate_primitive_.lastSegment().states.back().velocity;
            PathSegment terminal_loiter;
            generateCircle(end_position, end_velocity, goal_pos_, terminal_loiter);
            candidate_primitive_.appendSegment(terminal_loiter);

            reference_primitive_ = candidate_primitive_;
            next_state = query_state;
          }
          break;
        }
        case PLANNER_STATE::ROLLOUT: {
          planner_mode_ = PLANNER_MODE::ACTIVE_MAPPING;
          next_state = query_state;
          break;
        }
      }
      break;
    }
    case PLANNER_STATE::RETURN: {
      /// TODO: Check if we have a return position defined
      // Switch to Hold when segment has been completed
      int current_segment_idx = reference_primitive_.getCurrentSegmentIndex(vehicle_position_);
      bool is_last_segment = bool(current_segment_idx >= (reference_primitive_.segments.size() - 1));
      if (is_last_segment) {
        /// TODO: Clear candidate primitive
        candidate_primitive_.segments.clear();
        next_state = PLANNER_STATE::HOLD;
        found_return_solution_ = false;
        planner_mode_ = PLANNER_MODE::GLOBAL;
      }

      if (query_state == PLANNER_STATE::ABORT) {
        if (!reference_primitive_.getCurrentSegment(vehicle_position_).is_periodic) {
          next_state = PLANNER_STATE::ABORT;
          planner_mode_ = PLANNER_MODE::EMERGENCY_ABORT;
        }
      }
      break;
    }
    case PLANNER_STATE::ABORT: {
      int current_segment_idx = reference_primitive_.getCurrentSegmentIndex(vehicle_position_);
      bool is_last_segment = bool(current_segment_idx >= (reference_primitive_.segments.size() - 1));
      if (is_last_segment) {
        /// TODO: Clear candidate primitive
        candidate_primitive_.segments.clear();
        planner_mode_ = PLANNER_MODE::GLOBAL;
        found_solution_ = false;
        next_state = PLANNER_STATE::HOLD;
      }
    }
  }
  return next_state;
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

void TerrainPlanner::mavLocalPoseCallback(const geometry_msgs::PoseStamped &msg) {
  vehicle_attitude_(0) = msg.pose.orientation.w;
  vehicle_attitude_(1) = msg.pose.orientation.x;
  vehicle_attitude_(2) = msg.pose.orientation.y;
  vehicle_attitude_(3) = msg.pose.orientation.z;
}

void TerrainPlanner::mavGlobalPoseCallback(const sensor_msgs::NavSatFix &msg) {
  Eigen::Vector3d wgs84_vehicle_position;
  wgs84_vehicle_position(0) = msg.latitude;
  wgs84_vehicle_position(1) = msg.longitude;
  wgs84_vehicle_position(2) = msg.altitude;

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
}

void TerrainPlanner::mavtwistCallback(const geometry_msgs::TwistStamped &msg) {
  vehicle_velocity_ = toEigen(msg.twist.linear);
  // mavRate_ = toEigen(msg.twist.angular);
}

void TerrainPlanner::MapPublishOnce(const ros::Publisher &pub, const grid_map::GridMap &map) {
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(map, message);
  pub.publish(message);
}

void TerrainPlanner::publishPositionHistory(ros::Publisher &pub, const Eigen::Vector3d &position,
                                            std::vector<geometry_msgs::PoseStamped> &history_vector) {
  unsigned int posehistory_window_ = 200;
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
  msg.coordinate_frame = GlobalPositionTarget::FRAME_GLOBAL_INT;
  msg.type_mask = 0.0;
  msg.latitude = latitude;
  msg.longitude = longitude;
  msg.altitude = altitude;
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
  visualization_msgs::Marker marker = vector2ArrowsMsg(position, velocity, 0, Eigen::Vector3d(1.0, 0.0, 1.0));
  pub.publish(marker);
}

void TerrainPlanner::publishReferenceMarker(const ros::Publisher &pub, const Eigen::Vector3d &position,
                                            const Eigen::Vector3d &velocity, const double curvature) {
  Eigen::Vector3d scaled_velocity = 20.0 * velocity;
  visualization_msgs::Marker marker =
      vector2ArrowsMsg(position, scaled_velocity, 0, Eigen::Vector3d(0.0, 0.0, 1.0), "reference");

  pub.publish(marker);
}

void TerrainPlanner::publishPathSegments(ros::Publisher &pub, Path &trajectory) {
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
                                 Eigen::Vector3d color, std::string name_space) {
  visualization_msgs::Marker marker;
  marker = getGoalMarker(1, position, radius, color);
  marker.ns = name_space;
  pub.publish(marker);
}

void TerrainPlanner::publishRallyPoints(const ros::Publisher &pub, const std::vector<Eigen::Vector3d> &positions,
                                        const double radius, Eigen::Vector3d color, std::string name_space) {
  visualization_msgs::MarkerArray marker_array;
  std::vector<visualization_msgs::Marker> markers;
  int marker_id = 1;
  for (const auto &position : positions) {
    visualization_msgs::Marker marker;
    marker = getGoalMarker(marker_id, position, radius, color);
    marker.ns = name_space;
    markers.push_back(marker);
    marker_id++;
  }
  marker_array.markers = markers;
  pub.publish(marker_array);
}

visualization_msgs::Marker TerrainPlanner::getGoalMarker(const int id, const Eigen::Vector3d &position,
                                                         const double radius, const Eigen::Vector3d color) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.id = id;
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
  return marker;
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

void TerrainPlanner::mavMissionCallback(const mavros_msgs::WaypointListPtr &msg) {
  for (auto &waypoint : msg->waypoints) {
    if (waypoint.is_current) {
      if (waypoint.command == mavros_msgs::CommandCode::NAV_LOITER_UNLIM) {
        // Get Loiter position
        std::cout << "NAV Loiter Center" << std::endl;
        std::cout << " - x_lat : " << waypoint.x_lat << std::endl;
        std::cout << " - y_long: " << waypoint.y_long << std::endl;
        std::cout << " - alt   : " << waypoint.z_alt << std::endl;
        std::cout << " - Radius: " << waypoint.param3 << std::endl;
        double waypoint_altitude = waypoint.z_alt;
        const double loiter_radius = waypoint.param3;
        Eigen::Vector3d lv03_mission_loiter_center;
        GeoConversions::forward(waypoint.x_lat, waypoint.y_long, waypoint_altitude, lv03_mission_loiter_center.x(),
                                lv03_mission_loiter_center.y(), lv03_mission_loiter_center.z());
        std::cout << "mission_loiter_center_: " << lv03_mission_loiter_center.transpose() << std::endl;
        ESPG map_coordinate;
        Eigen::Vector3d map_origin;
        terrain_map_->getGlobalOrigin(map_coordinate, map_origin);

        if (map_coordinate == ESPG::WGS84) {
          GeoConversions::forward(map_origin(0), map_origin(1), map_origin(2), map_origin.x(), map_origin.y(),
                                  map_origin.z());
        }
        mission_loiter_center_ = lv03_mission_loiter_center - map_origin;
        mission_loiter_radius_ = loiter_radius;
        std::cout << "mission_loiter_center_: " << mission_loiter_center_.transpose() << std::endl;
      }
      break;
    }
  }
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
  bool result = terrain_map_->Load(map_path_, map_color_path_);
  std::cout << "[TerrainPlanner]   Computing distance transforms" << std::endl;
  terrain_map_->AddLayerDistanceTransform(min_elevation_, "distance_surface");
  terrain_map_->AddLayerDistanceTransform(max_elevation_, "max_elevation");
  std::cout << "[TerrainPlanner]   Computing horizontal distance transforms " << std::endl;
  terrain_map_->AddLayerHorizontalDistanceTransform(goal_radius_, "ics_+", "distance_surface");
  terrain_map_->AddLayerHorizontalDistanceTransform(-goal_radius_, "ics_-", "max_elevation");
  std::cout << "[TerrainPlanner]   Computing safety layers" << std::endl;
  terrain_map_->addLayerSafety("safety", "ics_+", "ics_-");

  if (result) {
    global_planner_->setBoundsFromMap(terrain_map_->getGridMap());
    global_planner_->setupProblem(start_pos_, goal_pos_, start_loiter_radius_);
    problem_updated_ = true;
    posehistory_vector_.clear();
    MapPublishOnce(grid_map_pub_, terrain_map_->getGridMap());
  }
  res.success = result;
  return true;
}

// void TerrainPlanner::dynamicReconfigureCallback(terrain_navigation_ros::HeightRateTuningConfig &config, uint32_t
// level)
// {
//   if (K_z_ != config.K_z) {
//     K_z_ = config.K_z;
//     ROS_INFO("Reconfigure request : K_z_  = %.2f  ", config.K_z);
//   }
//   if (cruise_speed_ != config.cruise_speed) {
//     cruise_speed_ = config.cruise_speed;
//     ROS_INFO("Reconfigure request : cruise_speed_  = %.2f  ", config.cruise_speed);
//   }
//   if (max_climb_rate_control_ != config.max_climb_rate) {
//     max_climb_rate_control_ = config.max_climb_rate;
//     ROS_INFO("Reconfigure request : max_climb_rate  = %.2f  ", config.max_climb_rate);
//   }
// }

bool TerrainPlanner::setMaxAltitudeCallback(planner_msgs::SetString::Request &req,
                                            planner_msgs::SetString::Response &res) {
  bool check_max_altitude = req.align;
  std::cout << "[TerrainPlanner] Max altitude constraint configured: " << check_max_altitude << std::endl;
  global_planner_->setMaxAltitudeCollisionChecks(check_max_altitude);
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
    publishGoal(candidate_goal_pub_, new_goal, goal_radius_, Eigen::Vector3d(0.0, 1.0, 0.0), "goal");
    return true;
  } else {
    res.success = false;
    publishGoal(candidate_goal_pub_, new_goal, goal_radius_, Eigen::Vector3d(1.0, 0.0, 0.0), "goal");
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
    publishGoal(candidate_start_pub_, new_start, start_loiter_radius_, Eigen::Vector3d(0.0, 1.0, 0.0), "start");
    return true;
  } else {
    res.success = false;
    publishGoal(candidate_start_pub_, new_start, start_loiter_radius_, Eigen::Vector3d(1.0, 0.0, 0.0), "start");
    return false;
  }
}

bool TerrainPlanner::setCurrentSegmentCallback(planner_msgs::SetService::Request &req,
                                               planner_msgs::SetService::Response &res) {
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  /// TODO: Get center of the last segment of the reference path
  if (!reference_primitive_.segments.empty()) {
    auto last_segment = reference_primitive_.lastSegment();
    if (last_segment.is_periodic) {
      /// TODO: Get the center of the circle
      Eigen::Vector3d segment_start = last_segment.states.front().position;
      Eigen::Vector3d segment_start_tangent = (last_segment.states.front().velocity).normalized();
      auto arc_center =
          PathSegment::getArcCenter(segment_start.head(2), segment_start_tangent.head(2), last_segment.curvature);
      Eigen::Vector3d candidate_start = Eigen::Vector3d(arc_center(0), arc_center(1), 0.0);
      Eigen::Vector3d new_start;
      bool is_safe = validatePosition(terrain_map_->getGridMap(), candidate_start, new_start);
      if (is_safe) {
        start_pos_ = new_start;
        /// TODO: Curvature sign seems to be the opposite from mission items
        start_loiter_radius_ = -1 / last_segment.curvature;
        res.success = true;
        publishGoal(candidate_start_pub_, new_start, goal_radius_, Eigen::Vector3d(0.0, 1.0, 0.0), "start");
        return true;
      } else {
        res.success = false;
        publishGoal(candidate_start_pub_, new_start, goal_radius_, Eigen::Vector3d(1.0, 0.0, 0.0), "start");
        return false;
      }
    } else {
      std::cout << "[TerrainPlanner] Last segment is not periodic" << std::endl;
    }
  }
  std::cout << "[TerrainPlanner] Could not select current segment, reference is empty" << std::endl;
  res.success = false;
  return true;
}

bool TerrainPlanner::setStartLoiterCallback(planner_msgs::SetService::Request &req,
                                            planner_msgs::SetService::Response &res) {
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  std::cout << "[TerrainPlanner] Current Loiter start: " << mission_loiter_center_.transpose() << std::endl;
  Eigen::Vector3d new_start;
  bool is_safe = validatePosition(terrain_map_->getGridMap(), mission_loiter_center_, new_start);
  if (is_safe) {
    start_pos_ = mission_loiter_center_;
    start_loiter_radius_ = mission_loiter_radius_;
    home_position_ = start_pos_;
    home_position_radius_ = start_loiter_radius_;
    res.success = true;
    publishGoal(candidate_start_pub_, start_pos_, std::abs(mission_loiter_radius_), Eigen::Vector3d(0.0, 1.0, 0.0),
                "start");
    return true;
  } else {
    res.success = false;
    publishGoal(candidate_start_pub_, start_pos_, std::abs(mission_loiter_radius_), Eigen::Vector3d(1.0, 0.0, 0.0),
                "start");
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
  global_planner_->setupProblem(start_pos_, goal_pos_, start_loiter_radius_);
  planner_mode_ = PLANNER_MODE::GLOBAL;
  res.success = true;
  return true;
}

bool TerrainPlanner::setPathCallback(planner_msgs::SetVector3::Request &req, planner_msgs::SetVector3::Response &res) {
  const std::lock_guard<std::mutex> lock(goal_mutex_);

  if (!candidate_primitive_.segments.empty()) {
    query_planner_state_ = PLANNER_STATE::NAVIGATE;
    res.success = true;
  } else {
    res.success = false;
  }
  return true;
}

bool TerrainPlanner::setPlannerStateCallback(planner_msgs::SetPlannerState::Request &req,
                                             planner_msgs::SetPlannerState::Response &res) {
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  int planner_state = static_cast<int>(req.state);
  switch (planner_state) {
    case (1): {
      query_planner_state_ = PLANNER_STATE::HOLD;
      res.success = true;
      break;
    }
    case (2): {
      query_planner_state_ = PLANNER_STATE::NAVIGATE;
      res.success = true;
      break;
    }
    case (3): {
      query_planner_state_ = PLANNER_STATE::ROLLOUT;
      res.success = true;
      break;
    }
    case (4): {
      query_planner_state_ = PLANNER_STATE::ABORT;
      res.success = true;
      break;
    }
    case (5): {
      query_planner_state_ = PLANNER_STATE::RETURN;
      res.success = true;
      break;
    }
    default: {
      res.success = false;
      break;
    }
  }
  std::cout << "planner state: " << planner_state << std::endl;
  return true;
}

void TerrainPlanner::generateCircle(const Eigen::Vector3d end_position, const Eigen::Vector3d end_velocity,
                                    const Eigen::Vector3d center_pos, PathSegment &trajectory) {
  Eigen::Vector3d radial_vector = (end_position - center_pos);
  radial_vector(2) = 0.0;  // Only consider horizontal loiters
  Eigen::Vector3d emergency_rates =
      20.0 * end_velocity.normalized().cross(radial_vector.normalized()) / radial_vector.norm();
  double horizon = 2 * M_PI / std::abs(emergency_rates(2));
  // Append a loiter at the end of the planned path
  trajectory = generateArcTrajectory(emergency_rates, horizon, end_position, end_velocity);
  trajectory.is_periodic = true;
  return;
}

PathSegment TerrainPlanner::generateArcTrajectory(Eigen::Vector3d rate, const double horizon,
                                                  Eigen::Vector3d current_pos, Eigen::Vector3d current_vel,
                                                  const double dt) {
  PathSegment trajectory;
  trajectory.states.clear();

  double time = 0.0;
  const double current_yaw = std::atan2(-1.0 * current_vel(1), current_vel(0));
  const double climb_rate = rate(1);
  trajectory.flightpath_angle = std::asin(climb_rate / cruise_speed_);
  /// TODO: Fix sign conventions for curvature
  trajectory.curvature = -rate(2) / cruise_speed_;
  trajectory.dt = dt;
  for (int i = 0; i < std::max(1.0, horizon / dt); i++) {
    if (std::abs(rate(2)) < 0.0001) {
      rate(2) > 0.0 ? rate(2) = 0.0001 : rate(2) = -0.0001;
    }
    double yaw = rate(2) * time + current_yaw;

    Eigen::Vector3d pos =
        cruise_speed_ / rate(2) *
            Eigen::Vector3d(std::sin(yaw) - std::sin(current_yaw), std::cos(yaw) - std::cos(current_yaw), 0) +
        Eigen::Vector3d(0, 0, climb_rate * time) + current_pos;
    Eigen::Vector3d vel = Eigen::Vector3d(cruise_speed_ * std::cos(yaw), -cruise_speed_ * std::sin(yaw), -climb_rate);
    const double roll = std::atan(rate(2) * cruise_speed_ / 9.81);
    const double pitch = std::atan(climb_rate / cruise_speed_);
    Eigen::Vector4d att = rpy2quaternion(roll, -pitch, -yaw);  // TODO: why the hell do you need to reverse signs?

    State state_vector;
    state_vector.position = pos;
    state_vector.velocity = vel;
    state_vector.attitude = att;
    trajectory.states.push_back(state_vector);

    time = time + dt;
  }
  return trajectory;
}

void TerrainPlanner::publishPath(const ros::Publisher &pub, Path &path) {
  planner_msgs::Path path_msg;
  for (const auto &path_segment : path.segments) {
    planner_msgs::PathSegment segment_msg;
    segment_msg.reached = path_segment.reached;
    segment_msg.segment_start = toVector3(path_segment.states.front().position);
    Eigen::Vector3d start_velocity = path_segment.states.front().velocity;
    segment_msg.segment_tangent = toVector3(start_velocity.normalized());
    segment_msg.periodic = path_segment.is_periodic;
    segment_msg.curvature.data = double(path_segment.curvature);
    double segment_length = path_segment.getLength();
    segment_msg.segment_length.data = double(segment_length);
    path_msg.segments.push_back(segment_msg);
  }
  pub.publish(path_msg);
  return;
}
