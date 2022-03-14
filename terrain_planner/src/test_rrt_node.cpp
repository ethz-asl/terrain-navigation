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

#include "terrain_navigation/terrain_map.h"
#include "terrain_planner/common.h"
#include "terrain_planner/terrain_ompl_rrt.h"

#include <grid_map_ros/GridMapRosConverter.hpp>

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

void publishPositionSetpoints(ros::Publisher& pub, const Eigen::Vector3d& position, const Eigen::Vector3d& velocity) {
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
  marker.pose.position.x = position(0);
  marker.pose.position.y = position(1);
  marker.pose.position.z = position(2);
  double yaw = std::atan2(velocity(1), velocity(0));
  marker.pose.orientation.w = std::cos(0.5 * yaw);
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = std::sin(0.5 * yaw);
  visualization_msgs::MarkerArray msg;

  pub.publish(marker);
}

geometry_msgs::Point toPoint(const Eigen::Vector3d& p) {
  geometry_msgs::Point position;
  position.x = p(0);
  position.y = p(1);
  position.z = p(2);
  return position;
}

void publishTree(ros::Publisher& pub, std::shared_ptr<ompl::base::PlannerData> planner_data,
                 ompl::OmplSetup& problem_setup) {
  visualization_msgs::MarkerArray marker_array;
  planner_data->decoupleFromPlanner();

  // allocate variables
  std::vector<unsigned int> edge_list;
  int edge_id = 0;
  int num_vertices = planner_data->numVertices();
  std::cout << "Number of vertices" << num_vertices << std::endl;

  // fill common variables of the marker message

  // Create states, a marker and a list to store edges
  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> vertex(problem_setup.getSpaceInformation());
  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> neighbor_vertex(problem_setup.getSpaceInformation());
  int marker_idx{0};
  for (int i = 0; i < num_vertices; i++) {
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
        points.push_back(toPoint(Eigen::Vector3d(vertex[0], vertex[1], vertex[2])));
        points.push_back(toPoint(Eigen::Vector3d(neighbor_vertex[0], neighbor_vertex[1], neighbor_vertex[2])));
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

  ros::Publisher start_pos_pub = nh.advertise<visualization_msgs::Marker>("start_position", 1, true);
  ros::Publisher goal_pos_pub = nh.advertise<visualization_msgs::Marker>("goal_position", 1, true);
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 1, true);
  ros::Publisher grid_map_pub_ = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  ros::Publisher trajectory_pub_ = nh.advertise<visualization_msgs::MarkerArray>("tree", 1, true);

  std::string map_path, color_file_path;
  nh_private.param<std::string>("map_path", map_path, "");
  nh_private.param<std::string>("color_file_path", color_file_path, "");

  std::shared_ptr<TerrainMap> terrain_map_ = std::make_shared<TerrainMap>();
  bool loaded = terrain_map_->initializeFromGeotiff(map_path, false);
  if (!color_file_path.empty()) {  // Load color layer if the color path is nonempty
    bool color_loaded = terrain_map_->addColorFromGeotiff(color_file_path);
  }
  terrain_map_->AddLayerDistanceTransform("distance_surface");

  std::shared_ptr<TerrainOmplRrt> planner = std::make_shared<TerrainOmplRrt>(terrain_map_->getGridMap());
  /// TODO: Get bounds from gridmap
  grid_map::GridMap& map = terrain_map_->getGridMap();
  const Eigen::Vector2d map_pos = map.getPosition();
  const double map_width_x = map.getLength().x();
  const double map_width_y = map.getLength().y();
  double roi_ratio = 0.5;
  Eigen::Vector3d lower_bounds{
      Eigen::Vector3d(-map_pos(0) - roi_ratio * map_width_x, map_pos(1) - roi_ratio * map_width_y, -100.0)};
  Eigen::Vector3d upper_bounds{
      Eigen::Vector3d(map_pos(0) + roi_ratio * map_width_x, map_pos(1) + roi_ratio * map_width_y, 800.0)};
  planner->setBounds(lower_bounds, upper_bounds);
  planner->setupProblem();
  std::vector<Eigen::Vector3d> path;
  double terrain_altitude{100.0};

  Eigen::Vector3d start{Eigen::Vector3d(-200.0, -200.0, 0.0)};
  start(2) = terrain_map_->getGridMap().atPosition("elevation", Eigen::Vector2d(start(0), start(1))) + terrain_altitude;
  Eigen::Vector3d goal{Eigen::Vector3d(300.0, 300.0, 0.0)};
  goal(2) = terrain_map_->getGridMap().atPosition("elevation", Eigen::Vector2d(goal(0), goal(1))) + terrain_altitude;
  planner->Solve(start, goal, path);

  while (true) {
    terrain_map_->getGridMap().setTimestamp(ros::Time::now().toNSec());
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(terrain_map_->getGridMap(), message);
    grid_map_pub_.publish(message);
    publishTrajectory(path_pub, path);
    publishPositionSetpoints(start_pos_pub, start, Eigen::Vector3d(15.0, 0.0, 0.0));
    publishPositionSetpoints(goal_pos_pub, goal, Eigen::Vector3d(15.0, 0.0, 0.0));
    publishTree(trajectory_pub_, planner->getPlannerData(), planner->getProblemSetup());
    ros::Duration(1.0).sleep();
  }
  ros::spin();
  return 0;
}
