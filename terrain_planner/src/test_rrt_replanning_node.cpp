/****************************************************************************
 *
 *   Copyright (c) 2021-2022 Jaeyoung Lim. All rights reserved.
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

void publishPositionSetpoints(const ros::Publisher& pub, const Eigen::Vector3d& position,
                              const Eigen::Vector3d& velocity) {
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::ARROW;
  marker.header.frame_id = "map";
  marker.id = 0;
  marker.action = visualization_msgs::Marker::DELETEALL;
  pub.publish(marker);

  marker.header.stamp = ros::Time::now();
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 10.0;
  marker.scale.y = 2.0;
  marker.scale.z = 2.0;
  marker.color.a = 0.5;  // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.pose.position = tf2::toMsg(position);
  tf2::Quaternion q;
  q.setRPY(0, 0, std::atan2(velocity.y(), velocity.x()));
  marker.pose.orientation = tf2::toMsg(q);
  visualization_msgs::MarkerArray msg;

  pub.publish(marker);
}

void publishTree(const ros::Publisher& pub, std::shared_ptr<ompl::base::PlannerData> planner_data,
                 std::shared_ptr<ompl::OmplSetup> problem_setup) {
  visualization_msgs::MarkerArray marker_array;
  planner_data->decoupleFromPlanner();

  // allocate variables
  std::vector<unsigned int> edge_list;

  // Create states, a marker and a list to store edges
  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> vertex(problem_setup->getSpaceInformation());
  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> neighbor_vertex(problem_setup->getSpaceInformation());
  size_t marker_idx{0};
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
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.scale.x = 10.0;
    marker.scale.y = 10.0;
    marker.scale.z = 10.0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = 0.5;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker_array.markers.push_back(marker);

    int num_edges = planner_data->getEdges(i, edge_list);
    if (num_edges > 0) {
      visualization_msgs::Marker edge_marker;
      edge_marker.header.stamp = ros::Time().now();
      edge_marker.header.frame_id = "map";
      edge_marker.id = marker_idx++;
      edge_marker.type = visualization_msgs::Marker::LINE_LIST;
      edge_marker.ns = "edge";
      std::vector<geometry_msgs::Point> points;
      for (unsigned int edge : edge_list) {
        neighbor_vertex = planner_data->getVertex(edge).getState();
        points.push_back(toMsg(Eigen::Vector3d(vertex[0], vertex[1], vertex[2])));
        points.push_back(toMsg(Eigen::Vector3d(neighbor_vertex[0], neighbor_vertex[1], neighbor_vertex[2])));
      }
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
  pub.publish(marker_array);
}

void publishTrajectory(ros::Publisher& pub, std::vector<Eigen::Vector3d> trajectory) {
  nav_msgs::Path msg;
  std::vector<geometry_msgs::PoseStamped> posestampedhistory_vector;
  Eigen::Vector4d orientation(1.0, 0.0, 0.0, 0.0);
  for (auto pos : trajectory) {
    posestampedhistory_vector.insert(posestampedhistory_vector.begin(), vector3d2PoseStampedMsg(pos, orientation));
  }
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.poses = posestampedhistory_vector;
  pub.publish(msg);
}

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
  terrain_map->AddLayerDistanceTransform("distance_surface");

  // Initialize planner with loaded terrain map
  auto planner = std::make_shared<TerrainOmplRrt>();
  planner->setMap(terrain_map);
  /// TODO: Get bounds from gridmap
  planner->setBoundsFromMap(terrain_map->getGridMap());
  planner->setupProblem();
  std::vector<Eigen::Vector3d> path;
  double terrain_altitude{100.0};

  Eigen::Vector3d start{Eigen::Vector3d(-200.0, -200.0, 0.0)};
  start(2) = terrain_map->getGridMap().atPosition("elevation", Eigen::Vector2d(start(0), start(1))) + terrain_altitude;
  Eigen::Vector3d start_vel{Eigen::Vector3d(10.0, 0.0, 0.0)};
  Eigen::Vector3d goal{Eigen::Vector3d(300.0, 300.0, 0.0)};
  goal(2) = terrain_map->getGridMap().atPosition("elevation", Eigen::Vector2d(goal(0), goal(1))) + terrain_altitude;

  // Repeatedly publish results
  while (true) {
    planner->Solve(start, start_vel, goal, path);
    terrain_map->getGridMap().setTimestamp(ros::Time::now().toNSec());
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(terrain_map->getGridMap(), message);
    grid_map_pub.publish(message);
    publishTrajectory(path_pub, path);
    publishPositionSetpoints(start_pos_pub, start, {15.0, 0.0, 0.0});
    publishPositionSetpoints(goal_pos_pub, goal, {15.0, 0.0, 0.0});
    publishTree(trajectory_pub, planner->getPlannerData(), planner->getProblemSetup());
    ros::Duration(1.0).sleep();
  }
  ros::spin();
  return 0;
}