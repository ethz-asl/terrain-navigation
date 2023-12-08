#ifndef TERRAIN_PLANNER_VISUALIZATION_H
#define TERRAIN_PLANNER_VISUALIZATION_H

#include <terrain_navigation/path.h>
#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Dense>
#include <fstream>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "terrain_planner/common.h"
#include "terrain_planner/ompl_setup.h"

void publishCandidateManeuvers(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub,
                               const std::vector<Path>& candidate_maneuvers,
                               bool visualize_invalid_trajectories = false);

void publishPositionSetpoints(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                              const Eigen::Vector3d& position, const Eigen::Vector3d& velocity,
                              const Eigen::Vector3d scale = Eigen::Vector3d(10.0, 2.0, 2.0));

void publishPath(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub, std::vector<Eigen::Vector3d> path,
                 Eigen::Vector3d color);

void publishTrajectory(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub, std::vector<Eigen::Vector3d> trajectory);

void publishTree(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub,
                 std::shared_ptr<ompl::base::PlannerData> planner_data, std::shared_ptr<ompl::OmplSetup> problem_setup);

#endif
