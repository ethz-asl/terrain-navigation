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
 * @brief ROS Node to test ompl
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <terrain_navigation/terrain_map.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include "terrain_planner/common.h"
#include "terrain_planner/terrain_ompl_rrt.h"
#include "terrain_planner/visualization.h"

void publishCircleSetpoints(const ros::Publisher& pub, const Eigen::Vector3d& position, const double radius) {
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.header.frame_id = "map";
  marker.id = 0;
  marker.header.stamp = ros::Time::now();
  std::vector<geometry_msgs::Point> points;
  for (double t = 0.0; t <= 1.0; t += 0.02) {
    geometry_msgs::Point point;
    point.x = position.x() + radius * std::cos(t * 2 * M_PI);
    point.y = position.y() + radius * std::sin(t * 2 * M_PI);
    point.z = position.z();
    points.push_back(point);
  }
  geometry_msgs::Point start_point;
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
  pub.publish(marker);
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
  double dt = 0.02;
  for (double t = 0.0; t <= 1.0 + dt; t += dt) {
    dubins_ss->interpolate(from, to, t, state);
    auto interpolated_state =
        Eigen::Vector3d(state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getX(),
                        state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getY(),
                        state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getZ());
    path.push_back(interpolated_state);
  }
}

double mod2pi(double x) { return x - 2 * M_PI * floor(x * (0.5 / M_PI)); }

int main(int argc, char** argv) {
  ros::init(argc, argv, "ompl_rrt_planner");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  // Initialize ROS related publishers for visualization
  auto start_pos_pub = nh.advertise<visualization_msgs::Marker>("start_position", 1, true);
  auto goal_pos_pub = nh.advertise<visualization_msgs::Marker>("goal_position", 1, true);
  auto path_pub = nh.advertise<nav_msgs::Path>("path", 1, true);
  auto grid_map_pub = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  auto trajectory_pub = nh.advertise<visualization_msgs::MarkerArray>("tree", 1, true);

  std::string map_path, color_file_path;
  nh_private.param<std::string>("map_path", map_path, "");
  nh_private.param<std::string>("color_file_path", color_file_path, "");

  // Load terrain map from defined tif paths
  auto terrain_map = std::make_shared<TerrainMap>();
  terrain_map->initializeFromGeotiff(map_path, false);
  if (!color_file_path.empty()) {  // Load color layer if the color path is nonempty
    terrain_map->addColorFromGeotiff(color_file_path);
  }
  terrain_map->AddLayerDistanceTransform(50.0, "distance_surface");
  terrain_map->AddLayerOffset(150.0, "max_elevation");

  std::vector<Eigen::Vector3d> path;
  double terrain_altitude{100.0};

  int num_experiments{1};
  for (int i = 0; i < num_experiments; i++) {
    // Initialize planner with loaded terrain map
    auto planner = std::make_shared<TerrainOmplRrt>();
    planner->setMap(terrain_map);
    /// TODO: Get bounds from gridmap
    planner->setBoundsFromMap(terrain_map->getGridMap());

    const Eigen::Vector2d map_pos = terrain_map->getGridMap().getPosition();
    const double map_width_x = terrain_map->getGridMap().getLength().x();
    const double map_width_y = terrain_map->getGridMap().getLength().y();

    Eigen::Vector3d start{Eigen::Vector3d(map_pos(0) + 0.3 * map_width_x, map_pos(1) - 0.3 * map_width_y, 0.0)};
    start(2) =
        terrain_map->getGridMap().atPosition("elevation", Eigen::Vector2d(start(0), start(1))) + terrain_altitude;
    // Eigen::Vector3d goal{Eigen::Vector3d(map_pos(0) + 0.3 * map_width_x, map_pos(1) - 0.35 * map_width_y, 0.0)};
    Eigen::Vector3d goal{Eigen::Vector3d(map_pos(0) + 0.3 * map_width_x, map_pos(1) + 0.3 * map_width_y, 0.0)};
    goal(2) = terrain_map->getGridMap().atPosition("elevation", Eigen::Vector2d(goal(0), goal(1))) + terrain_altitude;
    planner->setupProblem(start, goal);
    int max_iter = 10;
    int iter = 0;
    while (!planner->Solve(1.0, path)) {
      std::cout << "iter: " << iter << std::endl;
      if (iter > max_iter) {
        break;
      } else {
        iter++;
      }
    };

    double radius = 66.6667;

    // Repeatedly publish results
    terrain_map->getGridMap().setTimestamp(ros::Time::now().toNSec());
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(terrain_map->getGridMap(), message);
    grid_map_pub.publish(message);
    publishTrajectory(path_pub, path);

    /// TODO: Publish a circle instead of a goal marker!
    publishCircleSetpoints(start_pos_pub, start, radius);
    publishCircleSetpoints(goal_pos_pub, goal, radius);
    publishTree(trajectory_pub, planner->getPlannerData(), planner->getProblemSetup());
    ros::Duration(1.0).sleep();
  }
  ros::spin();
  return 0;
}
