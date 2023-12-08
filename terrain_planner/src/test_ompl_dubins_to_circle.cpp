/****************************************************************************
 *
 *   Copyright (c) 2022 Jaeyoung Lim, Autonomous Systems Lab,
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
 * @brief ROS Node to test ompl
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include <terrain_navigation/terrain_map.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/point.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "terrain_planner/common.h"
#include "terrain_planner/terrain_ompl_rrt.h"
#include "terrain_planner/visualization.h"

using namespace std::chrono_literals;

double mod2pi(double x) { return x - 2 * M_PI * floor(x * (0.5 / M_PI)); }

void publishCircleSetpoints(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                            const Eigen::Vector3d& position, const double radius) {
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = rclcpp::Clock().now();
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.header.frame_id = "map";
  marker.id = 0;
  marker.header.stamp = rclcpp::Clock().now();
  std::vector<geometry_msgs::msg::Point> points;
  for (double t = 0.0; t <= 1.0; t += 0.02) {
    geometry_msgs::msg::Point point;
    point.x = position.x() + radius * std::cos(t * 2 * M_PI);
    point.y = position.y() + radius * std::sin(t * 2 * M_PI);
    point.z = position.z();
    points.push_back(point);
  }
  geometry_msgs::msg::Point start_point;
  start_point.x = position.x() + radius * std::cos(0.0);
  start_point.y = position.y() + radius * std::sin(0.0);
  start_point.z = position.z();
  points.push_back(start_point);

  marker.points = points;
  marker.scale.x = 5.0;
  marker.scale.y = 5.0;
  marker.scale.z = 5.0;
  marker.color.a = 0.5;  // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  pub->publish(marker);
}

void getDubinsShortestPath(std::shared_ptr<fw_planning::spaces::DubinsAirplaneStateSpace>& dubins_ss,
                           const Eigen::Vector3d start_pos, const double start_yaw, const Eigen::Vector3d goal_pos,
                           const double goal_yaw, std::vector<Eigen::Vector3d>& path) {
  ompl::base::State* from = dubins_ss->allocState();
  from->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setX(start_pos.x());
  from->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setY(start_pos.y());
  from->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setZ(start_pos.z());
  from->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setYaw(start_yaw);

  ompl::base::State* to = dubins_ss->allocState();
  to->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setX(goal_pos.x());
  to->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setY(goal_pos.y());
  to->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setZ(goal_pos.z());
  to->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setYaw(goal_yaw);

  ompl::base::State* state = dubins_ss->allocState();
  for (double t = 0.0; t < 1.0; t += 0.02) {
    dubins_ss->interpolate(from, to, t, state);
    auto interpolated_state =
        Eigen::Vector3d(state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getX(),
                        state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getY(),
                        state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getZ());
    path.push_back(interpolated_state);
  }
}

double getDubinsTangentPoint(std::shared_ptr<fw_planning::spaces::DubinsAirplaneStateSpace>& dubins_ss,
                             const Eigen::Vector3d start_pos, const double start_yaw, const Eigen::Vector3d goal_pos,
                             const double goal_radius) {
  // References:
  //  [1] Chen, Zheng. "On Dubins paths to a circle." Automatica 117 (2020): 108996.
  //  [2] Manyam, Satyanarayana G., et al. "Shortest Dubins path to a circle." AIAA Scitech 2019 Forum. 2019.
  // The problem boils down on calculating the final yaw of the tangent point
  const double curvature = dubins_ss->getCurvature();
  double minimum_turn_radius = 1 / curvature;  // r

  Eigen::Vector3d error_pos = goal_pos - start_pos;
  double theta_final{0.0};
  // From [1] Lemma 6
  std::cout << "[DubinsToCircle] Goal Radius        : " << goal_radius << std::endl;
  std::cout << "[DubinsToCircle] Minimum Turn Radius: " << minimum_turn_radius << std::endl;
  bool csc_optimal = (error_pos.norm() > minimum_turn_radius + goal_radius);
  if (!csc_optimal) {
    /// TODO: Dubins to circle CCC case
    std::cout << "[DubinsToCircle] CCC path type could be optimal!" << std::endl;
    return theta_final;
  } else {
    std::cout << "[DubinsToCircle] CSC path type is optimal!" << std::endl;
    /// Angle of second straight segment
    /// TODO: How do we choose which alpha_s to use?
    // LSL Paths: From [2]
    double c = error_pos.x() * std::cos(start_yaw) - error_pos.y() * std::sin(-start_yaw) - error_pos.x();
    double d = error_pos.x() * std::sin(-start_yaw) + error_pos.y() * std::cos(-start_yaw) - error_pos.y();
    // double alpha_s = mod2pi(std::atan2(d - goal_radius, c));
    // RSL Paths
    double L_cc = std::sqrt(std::pow(d - minimum_turn_radius, 2) + c * c);
    double L_s = std::sqrt(L_cc * L_cc - 4 * minimum_turn_radius * minimum_turn_radius);
    double alpha_s = mod2pi(std::atan2(2 * minimum_turn_radius, L_s) -
                            std::atan2(error_pos.y(), error_pos.x() - goal_radius) + 0.5 * M_PI);
    alpha_s = alpha_s + start_yaw;
    // Calculate alpha_c
    double alpha_c{0.0};
    // From [1] Lemma 4
    bool external_tangent = (minimum_turn_radius >= goal_radius / 2.0);
    // From [1] Lemma 5
    external_tangent = external_tangent || minimum_turn_radius > goal_radius;
    if (external_tangent) {
      std::cout << "[DubinsToCircle]   - External tangent!" << std::endl;
      alpha_c = M_PI - std::acos(goal_radius / (goal_radius + minimum_turn_radius));
    } else {
      std::cout << "[DubinsToCircle]   - Internal tangent!" << std::endl;
      alpha_c = std::acos(goal_radius / (goal_radius + minimum_turn_radius));
    }
    // C3 = R
    // double theta_final = alpha_s - alpha_c;
    // C3 = L
    theta_final = alpha_s + alpha_c;
    return theta_final;
  }
}

class OmplRrtPlanner : public rclcpp::Node {
 public:
  OmplRrtPlanner() : Node("ompl_rrt_planner") {
    // Initialize ROS related publishers for visualization
    start_pos_pub = this->create_publisher<visualization_msgs::msg::Marker>("start_position", 1);
    goal_pos_pub = this->create_publisher<visualization_msgs::msg::Marker>("goal_position", 1);
    tangent_pos_pub = this->create_publisher<visualization_msgs::msg::Marker>("tangent_position", 1);
    path_pub = this->create_publisher<nav_msgs::msg::Path>("path", 1);
    path_pub_2 = this->create_publisher<nav_msgs::msg::Path>("candidate_path", 1);
    trajectory_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("tree", 1);

    timer = this->create_wall_timer(1s, std::bind(&OmplRrtPlanner::timer_callback, this));
  }

  void timer_callback() {
    auto dubins_ss = std::make_shared<fw_planning::spaces::DubinsAirplaneStateSpace>();
    Eigen::Vector3d start_pos(0.0, 0.0, 0.0);
    /// Goal circular radius
    Eigen::Vector3d goal_pos(400.0, 0.0, 0.0);
    double goal_radius = 66.6667;
    double tangent_yaw = getDubinsTangentPoint(dubins_ss, start_pos, start_yaw, goal_pos, goal_radius);

    /// TODO: Compare two different positions with same tangent yaw
    Eigen::Vector3d tangent_pos;
    tangent_pos << goal_pos.x() + goal_radius * std::cos(tangent_yaw - 0.5 * M_PI),
        goal_pos.y() + goal_radius * std::sin(tangent_yaw - 0.5 * M_PI), goal_pos.z();
    std::vector<Eigen::Vector3d> path_candidate_1;
    getDubinsShortestPath(dubins_ss, start_pos, start_yaw, tangent_pos, tangent_yaw, path_candidate_1);

    tangent_pos << goal_pos.x() + goal_radius * std::cos(tangent_yaw + 0.5 * M_PI),
        goal_pos.y() + goal_radius * std::sin(tangent_yaw + 0.5 * M_PI), goal_pos.z();
    std::vector<Eigen::Vector3d> path_candidate_2;
    getDubinsShortestPath(dubins_ss, start_pos, start_yaw, tangent_pos, tangent_yaw, path_candidate_2);

    // Visualize
    publishTrajectory(path_pub, path_candidate_1);
    publishTrajectory(path_pub_2, path_candidate_2);
    Eigen::Vector3d start_velocity(std::cos(start_yaw), std::sin(start_yaw), 0.0);
    publishPositionSetpoints(start_pos_pub, start_pos, start_velocity);
    Eigen::Vector3d tangent_velocity(std::cos(tangent_yaw), std::sin(tangent_yaw), 0.0);
    publishPositionSetpoints(tangent_pos_pub, tangent_pos, tangent_velocity);
    publishCircleSetpoints(goal_pos_pub, goal_pos, goal_radius);
    start_yaw += 0.1;
  }

 private:
  double start_yaw{M_PI_2};

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr start_pos_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_pos_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tangent_pos_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_2;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_pub;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto ompl_rrt_planner = std::make_shared<OmplRrtPlanner>();
  rclcpp::spin(ompl_rrt_planner);
  rclcpp::shutdown();
  return 0;
}
