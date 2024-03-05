/****************************************************************************
 *
 *   Copyright (c) 2021 Jaeyoung Lim, Autonomous Systems Lab,
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

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CameraImageCaptured.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/WaypointList.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Dense>
#include <mutex>

#include <terrain_navigation/profiler.h>

#include "terrain_navigation_ros/visualization.h"
#include "terrain_planner/common.h"
#include "terrain_planner/terrain_ompl_rrt.h"

#include <planner_msgs/SetPlannerState.h>
#include <planner_msgs/SetService.h>
#include <planner_msgs/SetString.h>
#include <planner_msgs/SetVector3.h>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <dynamic_reconfigure/server.h>
// #include "terrain_navigation_ros/HeightRateTuningConfig.h"

enum class PLANNER_MODE { ACTIVE_MAPPING, EMERGENCY_ABORT, EXHAUSTIVE, GLOBAL, GLOBAL_REPLANNING, RANDOM, RETURN };

enum class PLANNER_STATE { HOLD = 1, NAVIGATE = 2, ROLLOUT = 3, ABORT = 4, RETURN = 5 };

class TerrainPlanner {
 public:
  TerrainPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
  virtual ~TerrainPlanner();
  void Init();

 private:
  void cmdloopCallback(const ros::TimerEvent &event);
  void statusloopCallback(const ros::TimerEvent &event);
  void plannerloopCallback(const ros::TimerEvent &event);
  void publishTrajectory(std::vector<Eigen::Vector3d> trajectory);
  // States from vehicle
  void mavLocalPoseCallback(const geometry_msgs::PoseStamped &msg);
  void mavGlobalPoseCallback(const sensor_msgs::NavSatFix &msg);
  void mavtwistCallback(const geometry_msgs::TwistStamped &msg);
  void mavstateCallback(const mavros_msgs::State::ConstPtr &msg);
  void mavMissionCallback(const mavros_msgs::WaypointListPtr &msg);
  void mavImageCapturedCallback(const mavros_msgs::CameraImageCaptured::ConstPtr &msg);
  bool setLocationCallback(planner_msgs::SetString::Request &req, planner_msgs::SetString::Response &res);
  bool setMaxAltitudeCallback(planner_msgs::SetString::Request &req, planner_msgs::SetString::Response &res);
  bool setGoalCallback(planner_msgs::SetVector3::Request &req, planner_msgs::SetVector3::Response &res);
  bool setStartCallback(planner_msgs::SetVector3::Request &req, planner_msgs::SetVector3::Response &res);
  bool setCurrentSegmentCallback(planner_msgs::SetService::Request &req, planner_msgs::SetService::Response &res);
  bool setStartLoiterCallback(planner_msgs::SetService::Request &req, planner_msgs::SetService::Response &res);
  bool setPlanningCallback(planner_msgs::SetVector3::Request &req, planner_msgs::SetVector3::Response &res);
  bool setPlannerStateCallback(planner_msgs::SetPlannerState::Request &req,
                               planner_msgs::SetPlannerState::Response &res);
  bool setPathCallback(planner_msgs::SetVector3::Request &req, planner_msgs::SetVector3::Response &res);

  void MapPublishOnce(const ros::Publisher &pub, const grid_map::GridMap &map);
  void publishPositionHistory(ros::Publisher &pub, const Eigen::Vector3d &position,
                              std::vector<geometry_msgs::PoseStamped> &history_vector);
  void publishPositionSetpoints(const ros::Publisher &pub, const Eigen::Vector3d &position,
                                const Eigen::Vector3d &velocity, const double curvature);
  void publishGlobalPositionSetpoints(const ros::Publisher &pub, const double latitude, const double longitude,
                                      const double altitude, const Eigen::Vector3d &velocity, const double curvature);
  void publishReferenceMarker(const ros::Publisher &pub, const Eigen::Vector3d &position,
                              const Eigen::Vector3d &velocity, const double curvature);
  void publishVelocityMarker(const ros::Publisher &pub, const Eigen::Vector3d &position,
                             const Eigen::Vector3d &velocity);
  void publishPathSetpoints(const Eigen::Vector3d &position, const Eigen::Vector3d &velocity);
  void publishPathSegments(ros::Publisher &pub, Path &trajectory);
  void publishGoal(const ros::Publisher &pub, const Eigen::Vector3d &position, const double radius,
                   Eigen::Vector3d color = Eigen::Vector3d(1.0, 1.0, 0.0), std::string name_space = "goal");
  void publishPath(const ros::Publisher &pub, Path &path);
  void publishRallyPoints(const ros::Publisher &pub, const std::vector<Eigen::Vector3d> &positions, const double radius,
                          Eigen::Vector3d color = Eigen::Vector3d(1.0, 1.0, 0.0),
                          std::string name_space = "rallypoints");
  visualization_msgs::Marker getGoalMarker(const int id, const Eigen::Vector3d &position, const double radius,
                                           const Eigen::Vector3d color);
  void generateCircle(const Eigen::Vector3d end_position, const Eigen::Vector3d end_velocity,
                      const Eigen::Vector3d center_pos, PathSegment &trajectory);
  PathSegment generateArcTrajectory(Eigen::Vector3d rates, const double horizon, Eigen::Vector3d current_pos,
                                    Eigen::Vector3d current_vel, const double dt = 0.1);
  // void dynamicReconfigureCallback(terrain_navigation_ros::HeightRateTuningConfig &config, uint32_t level);

  void printPlannerState(PLANNER_STATE state) {
    switch (state) {
      case PLANNER_STATE::HOLD:  // Fallthrough
        std::cout << "PLANNER_STATE::HOLD" << std::endl;
        break;
      case PLANNER_STATE::NAVIGATE:  // Fallthrough
        std::cout << "PLANNER_STATE::NAVIGATE" << std::endl;
        break;
      case PLANNER_STATE::ROLLOUT:
        std::cout << "PLANNER_STATE::ROLLOUT" << std::endl;
        break;
      case PLANNER_STATE::ABORT:
        std::cout << "PLANNER_STATE::ABORT" << std::endl;
        break;
    }
  }

  Eigen::Vector4d rpy2quaternion(double roll, double pitch, double yaw);

  PLANNER_STATE finiteStateMachine(const PLANNER_STATE current_state, const PLANNER_STATE query_state);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher vehicle_path_pub_;
  ros::Publisher grid_map_pub_;
  ros::Publisher vehicle_pose_pub_;
  ros::Publisher camera_pose_pub_;
  ros::Publisher posehistory_pub_;
  ros::Publisher referencehistory_pub_;
  ros::Publisher global_position_setpoint_pub_;
  ros::Publisher position_target_pub_;
  ros::Publisher path_target_pub_;
  ros::Publisher planner_status_pub_;
  ros::Publisher goal_pub_;
  ros::Publisher rallypoint_pub_;
  ros::Publisher candidate_goal_pub_;
  ros::Publisher candidate_start_pub_;
  ros::Publisher tree_pub_;
  ros::Publisher vehicle_velocity_pub_;
  ros::Publisher path_segment_pub_;
  ros::Publisher reference_path_pub_;
  ros::Subscriber mavlocalpose_sub_;
  ros::Subscriber mavglobalpose_sub_;
  ros::Subscriber mavtwist_sub_;
  ros::Subscriber mavstate_sub_;
  ros::Subscriber mavmission_sub_;

  ros::ServiceServer setlocation_serviceserver_;
  ros::ServiceServer setmaxaltitude_serviceserver_;
  ros::ServiceServer setgoal_serviceserver_;
  ros::ServiceServer setstart_serviceserver_;
  ros::ServiceServer setstartloiter_serviceserver_;
  ros::ServiceServer setplanning_serviceserver_;
  ros::ServiceServer updatepath_serviceserver_;
  ros::ServiceServer setcurrentsegment_serviceserver_;
  ros::ServiceServer setplannerstate_service_server_;

  ros::Timer cmdloop_timer_, statusloop_timer_, plannerloop_timer_;
  ros::Time plan_time_;
  ros::Time last_triggered_time_;
  Eigen::Vector3d goal_pos_{Eigen::Vector3d(0.0, 0.0, 20.0)};
  Eigen::Vector3d start_pos_{Eigen::Vector3d(0.0, 0.0, 20.0)};
  Eigen::Vector3d home_position_{Eigen::Vector3d(0.0, 0.0, 20.0)};
  double home_position_radius_{0.0};
  Eigen::Vector3d tracking_error_{Eigen::Vector3d::Zero()};
  ros::CallbackQueue plannerloop_queue_;
  ros::CallbackQueue statusloop_queue_;
  ros::CallbackQueue cmdloop_queue_;
  std::unique_ptr<ros::AsyncSpinner> plannerloop_spinner_;
  std::unique_ptr<ros::AsyncSpinner> statusloop_spinner_;
  std::unique_ptr<ros::AsyncSpinner> cmdloop_spinner_;

  // dynamic_reconfigure::Server<terrain_navigation_ros::HeightRateTuningConfig> server;
  // dynamic_reconfigure::Server<terrain_navigation_ros::HeightRateTuningConfig>::CallbackType f;

  PLANNER_MODE planner_mode_{PLANNER_MODE::EXHAUSTIVE};
  PLANNER_STATE planner_state_{PLANNER_STATE::HOLD};
  PLANNER_STATE query_planner_state_{PLANNER_STATE::HOLD};

  std::shared_ptr<TerrainMap> terrain_map_;

  std::shared_ptr<fw_planning::spaces::DubinsAirplaneStateSpace> dubins_state_space_;
  // std::shared_ptr<ViewUtilityMap> viewutility_map_;
  std::shared_ptr<TerrainOmplRrt> global_planner_;
  std::shared_ptr<Profiler> planner_profiler_;
  Path reference_primitive_;
  Path candidate_primitive_;
  Path rollout_primitive_;
  mavros_msgs::State current_state_;
  std::optional<GeographicLib::LocalCartesian> enu_;

  std::mutex goal_mutex_;  // protects g_i

  std::vector<Eigen::Vector3d> vehicle_position_history_;
  std::vector<geometry_msgs::PoseStamped> posehistory_vector_;
  std::vector<geometry_msgs::PoseStamped> referencehistory_vector_;
  Eigen::Vector3d vehicle_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vehicle_velocity_{Eigen::Vector3d::Zero()};
  Eigen::Vector4d vehicle_attitude_{Eigen::Vector4d(1.0, 0.0, 0.0, 0.0)};
  Eigen::Vector3d last_planning_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d previous_start_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d previous_return_start_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d mission_loiter_center_{Eigen::Vector3d::Zero()};

  std::vector<Eigen::Vector3d> rally_points;

  // Altitude tracking loop parameters
  /// TODO: This needs to be handed over to TECS
  double K_z_{2.0};
  double cruise_speed_{20.0};
  double max_climb_rate_control_{5.0};

  std::string map_path_{};
  std::string map_color_path_{};
  std::string mesh_resource_path_{};
  std::string resource_path_{};
  double max_elevation_{120.0};
  double min_elevation_{50.0};
  double goal_radius_{66.67};
  double planner_time_budget_{30.0};
  double mission_loiter_radius_{66.67};
  double start_loiter_radius_{66.67};
  bool map_initialized_{false};
  bool planner_enabled_{false};
  bool problem_updated_{true};
  bool found_solution_{false};
  bool found_return_solution_{false};
};

#endif
