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

#include <terrain_navigation/profiler.h>
#include <terrain_navigation/viewpoint.h>
#include <terrain_planner/common.h>
#include <terrain_planner/terrain_ompl_rrt.h>
#include <terrain_planner/visualization.h>

#include <Eigen/Dense>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <geographic_msgs/msg/geo_point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <mavros_msgs/msg/camera_image_captured.hpp>
#include <mavros_msgs/msg/global_position_target.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/trajectory.hpp>
#include <mavros_msgs/msg/waypoint_list.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mutex>
#include <nav_msgs/msg/path.hpp>
#include <planner_msgs/msg/navigation_status.hpp>
#include <planner_msgs/srv/set_planner_state.hpp>
#include <planner_msgs/srv/set_service.hpp>
#include <planner_msgs/srv/set_string.hpp>
#include <planner_msgs/srv/set_vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <visualization_msgs/msg/marker.hpp>

enum class PLANNER_MODE { ACTIVE_MAPPING, EMERGENCY_ABORT, EXHAUSTIVE, GLOBAL, GLOBAL_REPLANNING, RANDOM, RETURN };

enum class PLANNER_STATE { HOLD = 1, NAVIGATE = 2, ROLLOUT = 3, ABORT = 4, RETURN = 5 };

class TerrainPlanner : public rclcpp::Node {
 public:
  TerrainPlanner();

  void init();

 private:
  // void cmdloopCallback(const ros::TimerEvent &event);
  // void statusloopCallback(const ros::TimerEvent &event);
  // void plannerloopCallback(const ros::TimerEvent &event);
  void cmdloopCallback();
  void statusloopCallback();
  void plannerloopCallback();

  void publishTrajectory(std::vector<Eigen::Vector3d> trajectory);

  // States from vehicle
  void mavLocalPoseCallback(const geometry_msgs::msg::PoseStamped &msg);
  void mavGlobalPoseCallback(const sensor_msgs::msg::NavSatFix &msg);
  void mavtwistCallback(const geometry_msgs::msg::TwistStamped &msg);
  void mavstateCallback(const mavros_msgs::msg::State &msg);
  void mavGlobalOriginCallback(const geographic_msgs::msg::GeoPointStamped &msg);
  void mavMissionCallback(const mavros_msgs::msg::WaypointList &msg);
  void mavImageCapturedCallback(const mavros_msgs::msg::CameraImageCaptured &msg);

  bool setLocationCallback(const std::shared_ptr<planner_msgs::srv::SetString::Request> req,
                           std::shared_ptr<planner_msgs::srv::SetString::Response> res);
  bool setMaxAltitudeCallback(const std::shared_ptr<planner_msgs::srv::SetString::Request> req,
                              std::shared_ptr<planner_msgs::srv::SetString::Response> res);
  bool setGoalCallback(const std::shared_ptr<planner_msgs::srv::SetVector3::Request> req,
                       std::shared_ptr<planner_msgs::srv::SetVector3::Response> res);
  bool setStartCallback(const std::shared_ptr<planner_msgs::srv::SetVector3::Request> req,
                        std::shared_ptr<planner_msgs::srv::SetVector3::Response> res);
  bool setCurrentSegmentCallback(const std::shared_ptr<planner_msgs::srv::SetService::Request> req,
                                 std::shared_ptr<planner_msgs::srv::SetService::Response> res);
  bool setStartLoiterCallback(const std::shared_ptr<planner_msgs::srv::SetService::Request> req,
                              std::shared_ptr<planner_msgs::srv::SetService::Response> res);
  bool setPlanningCallback(const std::shared_ptr<planner_msgs::srv::SetVector3::Request> req,
                           std::shared_ptr<planner_msgs::srv::SetVector3::Response> res);
  bool setPlannerStateCallback(const std::shared_ptr<planner_msgs::srv::SetPlannerState::Request> req,
                               std::shared_ptr<planner_msgs::srv::SetPlannerState::Response> res);
  bool setPathCallback(const std::shared_ptr<planner_msgs::srv::SetVector3::Request> req,
                       std::shared_ptr<planner_msgs::srv::SetVector3::Response> res);

  void MapPublishOnce(rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub, const grid_map::GridMap &map);
  void publishPositionHistory(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr, const Eigen::Vector3d &position,
                              std::vector<geometry_msgs::msg::PoseStamped> &history_vector);
  void publishPositionSetpoints(rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr pub,
                                const Eigen::Vector3d &position, const Eigen::Vector3d &velocity,
                                const double curvature);
  void publishGlobalPositionSetpoints(rclcpp::Publisher<mavros_msgs::msg::GlobalPositionTarget>::SharedPtr pub,
                                      const double latitude, const double longitude, const double altitude,
                                      const Eigen::Vector3d &velocity, const double curvature);
  void publishReferenceMarker(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                              const Eigen::Vector3d &position, const Eigen::Vector3d &velocity, const double curvature);
  void publishVelocityMarker(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                             const Eigen::Vector3d &position, const Eigen::Vector3d &velocity);
  void publishPathSegments(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr, Path &trajectory);
  void publishGoal(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub, const Eigen::Vector3d &position,
                   const double radius, Eigen::Vector3d color = Eigen::Vector3d(1.0, 1.0, 0.0),
                   std::string name_space = "goal");
  void publishRallyPoints(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub,
                          const std::vector<Eigen::Vector3d> &positions, const double radius,
                          Eigen::Vector3d color = Eigen::Vector3d(1.0, 1.0, 0.0),
                          std::string name_space = "rallypoints");
  // Create a goal marker, a circle located at position with given radius and color.
  visualization_msgs::msg::Marker getGoalMarker(const int id, const Eigen::Vector3d &position, const double radius,
                                                const Eigen::Vector3d color);
  void generateCircle(const Eigen::Vector3d end_position, const Eigen::Vector3d end_velocity,
                      const Eigen::Vector3d center_pos, PathSegment &trajectory);
  PathSegment generateArcTrajectory(Eigen::Vector3d rates, const double horizon, Eigen::Vector3d current_pos,
                                    Eigen::Vector3d current_vel, const double dt = 0.1);

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
      case PLANNER_STATE::RETURN:
        std::cout << "PLANNER_STATE::RETURN" << std::endl;
        break;
    }
  }

  Eigen::Vector4d rpy2quaternion(double roll, double pitch, double yaw);

  PLANNER_STATE finiteStateMachine(const PLANNER_STATE current_state, const PLANNER_STATE query_state);

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr vehicle_path_pub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vehicle_pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr camera_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr posehistory_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr referencehistory_pub_;
  rclcpp::Publisher<mavros_msgs::msg::GlobalPositionTarget>::SharedPtr global_position_setpoint_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr position_target_pub_;
  rclcpp::Publisher<planner_msgs::msg::NavigationStatus>::SharedPtr planner_status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rallypoint_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr candidate_goal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr candidate_start_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viewpoint_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr planned_viewpoint_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tree_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vehicle_velocity_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_segment_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mavlocalpose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr mavglobalpose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr mavtwist_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavstate_sub_;
  rclcpp::Subscription<mavros_msgs::msg::WaypointList>::SharedPtr mavmission_sub_;
  rclcpp::Subscription<geographic_msgs::msg::GeoPointStamped>::SharedPtr global_origin_sub_;
  rclcpp::Subscription<mavros_msgs::msg::CameraImageCaptured>::SharedPtr image_captured_sub_;

  rclcpp::Service<planner_msgs::srv::SetString>::SharedPtr setlocation_serviceserver_;
  rclcpp::Service<planner_msgs::srv::SetString>::SharedPtr setmaxaltitude_serviceserver_;
  rclcpp::Service<planner_msgs::srv::SetVector3>::SharedPtr setgoal_serviceserver_;
  rclcpp::Service<planner_msgs::srv::SetVector3>::SharedPtr setstart_serviceserver_;
  rclcpp::Service<planner_msgs::srv::SetService>::SharedPtr setstartloiter_serviceserver_;
  rclcpp::Service<planner_msgs::srv::SetVector3>::SharedPtr setplanning_serviceserver_;
  rclcpp::Service<planner_msgs::srv::SetVector3>::SharedPtr updatepath_serviceserver_;
  rclcpp::Service<planner_msgs::srv::SetService>::SharedPtr setcurrentsegment_serviceserver_;
  rclcpp::Service<planner_msgs::srv::SetPlannerState>::SharedPtr setplannerstate_service_server_;

  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr msginterval_serviceclient_;

  // ros::Timer cmdloop_timer_, statusloop_timer_, plannerloop_timer_;
  rclcpp::TimerBase::SharedPtr cmdloop_timer_;
  rclcpp::TimerBase::SharedPtr statusloop_timer_;
  rclcpp::TimerBase::SharedPtr plannerloop_timer_;

  rclcpp::Time plan_time_;
  rclcpp::Time last_triggered_time_;
  Eigen::Vector3d goal_pos_{Eigen::Vector3d(0.0, 0.0, 20.0)};
  Eigen::Vector3d start_pos_{Eigen::Vector3d(0.0, 0.0, 20.0)};
  Eigen::Vector3d home_position_{Eigen::Vector3d(0.0, 0.0, 20.0)};
  double home_position_radius_{0.0};
  Eigen::Vector3d tracking_error_{Eigen::Vector3d::Zero()};
  // ros::CallbackQueue plannerloop_queue_;
  // ros::CallbackQueue statusloop_queue_;
  // ros::CallbackQueue cmdloop_queue_;
  // std::unique_ptr<ros::AsyncSpinner> plannerloop_spinner_;
  // std::unique_ptr<ros::AsyncSpinner> statusloop_spinner_;
  // std::unique_ptr<ros::AsyncSpinner> cmdloop_spinner_;
  // std::shared_ptr<rclcpp::Node> plannerloop_node_;
  // std::shared_ptr<rclcpp::Node> statusloop_node_;
  // std::shared_ptr<rclcpp::Node> cmdloop_node_;
  rclcpp::executors::SingleThreadedExecutor cmdloop_executor_;
  rclcpp::executors::SingleThreadedExecutor statusloop_executor_;
  rclcpp::executors::SingleThreadedExecutor plannerloop_executor_;

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
  mavros_msgs::msg::State current_state_;
  std::optional<GeographicLib::LocalCartesian> enu_;

  std::mutex goal_mutex_;  // protects g_i

  std::vector<Eigen::Vector3d> vehicle_position_history_;
  std::vector<ViewPoint> added_viewpoint_list;
  std::vector<ViewPoint> planned_viewpoint_list;
  std::vector<geometry_msgs::msg::PoseStamped> posehistory_vector_;
  std::vector<geometry_msgs::msg::PoseStamped> referencehistory_vector_;
  std::vector<ViewPoint> viewpoints_;
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

  // Altitude controller P gain.
  double K_z_{0.5};
  // Trim (cruise) airpseed. Set to PX4 FW_AIRSPD_TRIM.
  double cruise_speed_{15.0};
  // Altitude controller max climb rate. Set to PX4 FW_T_CLMB_R_SP.
  double max_climb_rate_control_{3.0};

  std::string map_path_{};
  std::string map_color_path_{};
  std::string mesh_resource_path_{};
  std::string resource_path_{};
  double max_elevation_{120.0};
  double min_elevation_{50.0};
  double goal_radius_{66.67};
  double local_origin_altitude_{0.0};
  double local_origin_latitude_{0.0};
  double local_origin_longitude_{0.0};
  double planner_time_budget_{30.0};
  double mission_loiter_radius_{66.67};
  double start_loiter_radius_{66.67};
  bool local_origin_received_{false};
  bool map_initialized_{false};
  bool planner_enabled_{false};
  bool problem_updated_{true};
  bool found_solution_{false};
  bool found_return_solution_{false};
};

#endif
