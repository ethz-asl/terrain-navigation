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
 * @brief ROS Node to test ompl
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <any>

#include <terrain_navigation/terrain_map.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include "terrain_navigation/data_logger.h"
#include "terrain_navigation_ros/geo_conversions.h"
#include "terrain_navigation_ros/visualization.h"
#include "terrain_planner/common.h"
#include "terrain_planner/terrain_ompl_rrt.h"
#include "terrain_abort_planner/mission_io.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

void publishPathSegments(ros::Publisher& pub, Path& trajectory,
                         Eigen::Vector3d color = Eigen::Vector3d(0.0, 1.0, 0.0)) {
  visualization_msgs::MarkerArray msg;

  std::vector<visualization_msgs::Marker> marker;
  visualization_msgs::Marker mark;
  mark.action = visualization_msgs::Marker::DELETEALL;
  marker.push_back(mark);
  msg.markers = marker;
  pub.publish(msg);

  std::vector<visualization_msgs::Marker> segment_markers;
  int i = 0;
  for (auto& segment : trajectory.segments) {
    segment_markers.insert(segment_markers.begin(), trajectory2MarkerMsg(segment, i++, color));
    segment_markers.insert(segment_markers.begin(),
                           vector2ArrowsMsg(segment.position().front(), 5.0 * segment.velocity().front(), i++, color));
    segment_markers.insert(segment_markers.begin(), point2MarkerMsg(segment.position().back(), i++, color));
  }
  msg.markers = segment_markers;
  pub.publish(msg);
}

visualization_msgs::Marker getGoalMarker(const int id, const Eigen::Vector3d& position, const double radius,
                                         const Eigen::Vector3d color) {
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

void publishRallyPoints(const ros::Publisher& pub, const std::vector<Eigen::Vector3d>& positions, const double radius,
                        Eigen::Vector3d color = Eigen::Vector3d(1.0, 1.0, 0.0),
                        std::string name_space = "rallypoints") {
  visualization_msgs::MarkerArray marker_array;
  std::vector<visualization_msgs::Marker> markers;
  int marker_id = 1;
  for (const auto& position : positions) {
    visualization_msgs::Marker marker;
    marker = getGoalMarker(marker_id, position, radius, color);
    marker.ns = name_space;
    markers.push_back(marker);
    marker_id++;
  }
  marker_array.markers = markers;
  pub.publish(marker_array);
}

void publishGeoFence(const ros::Publisher& pub, const std::vector<Eigen::Vector2d>& geofence_polygon,
                     Eigen::Vector3d color = Eigen::Vector3d(1.0, 1.0, 0.0), std::string name_space = "geofence") {
  visualization_msgs::MarkerArray marker_array;
  std::vector<visualization_msgs::Marker> markers;
  for (int marker_id = 0; marker_id < geofence_polygon.size() - 1; marker_id++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    std::vector<geometry_msgs::Point> points;
    for (auto vertex : geofence_polygon) {
      geometry_msgs::Point point;
      point.x = vertex[0];
      point.y = vertex[1];
      point.z = 500.0;
      points.push_back(point);
    }
    points.push_back(points.front());  // Close the polygon
    marker.points = points;
    marker.scale.x = 10.0;
    marker.scale.y = 10.0;
    marker.scale.z = 10.0;
    marker.color.a = 0.5;  // Don't forget to set the alpha!
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.color.r = color(0);
    marker.color.g = color(1);
    marker.color.b = color(2);
    marker.ns = name_space;
    markers.push_back(marker);
  }
  marker_array.markers = markers;
  pub.publish(marker_array);
}

void publishWaypoints(const ros::Publisher& pub, const std::vector<Eigen::Vector3d>& geofence_polygon,
                      Eigen::Vector3d color = Eigen::Vector3d(0.0, 0.0, 1.0), std::string name_space = "waypoint") {
  visualization_msgs::MarkerArray marker_array;
  std::vector<visualization_msgs::Marker> markers;
  for (int marker_id = 0; marker_id < geofence_polygon.size() - 1; marker_id++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    std::vector<geometry_msgs::Point> points;
    for (auto vertex : geofence_polygon) {
      geometry_msgs::Point point;
      point.x = vertex[0];
      point.y = vertex[1];
      point.z = vertex[2];
      points.push_back(point);
    }
    points.push_back(points.front());  // Close the polygon
    marker.points = points;
    marker.scale.x = 10.0;
    marker.scale.y = 10.0;
    marker.scale.z = 10.0;
    marker.color.a = 0.5;  // Don't forget to set the alpha!
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.color.r = color(0);
    marker.color.g = color(1);
    marker.color.b = color(2);
    marker.ns = name_space;
    markers.push_back(marker);
  }
  marker_array.markers = markers;
  pub.publish(marker_array);
}

void publishGridMap(const ros::Publisher& pub, const grid_map::GridMap& map) {
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(map, message);
  pub.publish(message);
}


double mod2pi(double x) { return x - 2 * M_PI * floor(x * (0.5 / M_PI)); }

PathSegment generateArcTrajectory(Eigen::Vector3d rate, const double horizon, Eigen::Vector3d current_pos,
                                  Eigen::Vector3d current_vel, const double dt = 0.1) {
  PathSegment trajectory;
  trajectory.states.clear();

  double cruise_speed_{20.0};

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

int main(int argc, char** argv) {
  ros::init(argc, argv, "ompl_rrt_planner");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  // Initialize ROS related publishers for visualization
  auto grid_map_pub = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  auto trajectory_pub = nh.advertise<visualization_msgs::MarkerArray>("tree", 1, true);
  auto abort_path_pub = nh.advertise<visualization_msgs::MarkerArray>("abort_path_segments", 1, true);
  auto mission_path_pub = nh.advertise<visualization_msgs::MarkerArray>("mission_path_segments", 1, true);
  auto rallypoint_pub = nh.advertise<visualization_msgs::MarkerArray>("rallypoints_marker", 1);
  auto geofence_pub = nh.advertise<visualization_msgs::MarkerArray>("geofence_marker", 1);
  auto waypoint_pub = nh.advertise<visualization_msgs::MarkerArray>("waypoint_marker", 1);

  std::string map_path, color_file_path, output_directory, location, mission_file_path;
  nh_private.param<std::string>("map_path", map_path, "");
  nh_private.param<std::string>("color_file_path", color_file_path, "");
  nh_private.param<std::string>("location", location, "");
  nh_private.param<std::string>("mission_file_path", mission_file_path, "");
  nh_private.param<std::string>("output_directory", output_directory, "");

  // Initialize data logger for recording
  auto data_logger = std::make_shared<DataLogger>();
  data_logger->setKeys({"x", "y", "z"});

  // Load terrain map from defined tif paths
  auto terrain_map = std::make_shared<TerrainMap>();
  terrain_map->initializeFromGeotiff(map_path);
  if (!color_file_path.empty()) {  // Load color layer if the color path is nonempty
    terrain_map->addColorFromGeotiff(color_file_path);
  }
  terrain_map->AddLayerDistanceTransform(50.0, "distance_surface");
  terrain_map->AddLayerDistanceTransform(120.0, "max_elevation");
  double radius = 66.667;
  terrain_map->AddLayerHorizontalDistanceTransform(radius, "ics_+", "distance_surface");
  terrain_map->AddLayerHorizontalDistanceTransform(-radius, "ics_-", "max_elevation");

  // Parse mission file to get geofence information
  std::vector<Eigen::Vector2d> geofence_polygon;
  std::ifstream f(mission_file_path);
  json data = json::parse(f);
  json polygons = data["geoFence"]["polygons"];
  std::cout << "Number of vertices: " << polygons[0]["polygon"].size() << std::endl;
  for (auto polygon_vertex : polygons[0]["polygon"]) {
    Eigen::Vector2d vertex_wgs84 = Eigen::Vector2d(polygon_vertex[0], polygon_vertex[1]);
    // Convert vertex into lv03
    Eigen::Vector3d vertex_lv03;
    GeoConversions::forward(vertex_wgs84[0], vertex_wgs84[1], 0.0, vertex_lv03.x(), vertex_lv03.y(),
                            vertex_lv03.z());  /// FIXME: Assuming zero AMSL

    // Convert vertex into local frame (Relative to map origin)
    ESPG map_coordinate;
    Eigen::Vector3d map_origin;
    terrain_map->getGlobalOrigin(map_coordinate, map_origin);
    Eigen::Vector3d vertex_local = vertex_lv03 - map_origin;
    geofence_polygon.push_back(vertex_local.head(2));
  }

  for (auto vertex : geofence_polygon) {
    std::cout << "Parsed Geofence vertex: " << vertex.transpose() << std::endl;
  }

  // Parse mission to generate reference path
  std::vector<Eigen::Vector3d> waypoint_list;
  json mission = data["mission"]["items"];
  for (auto mission_item : mission) {
    std::cout << "Mission Item: " << mission_item << std::endl;
    int command = mission_item["command"];
    std::cout << "  -  command : " << command << std::endl;
    switch (command) {
      case 16: {  // waypoint
        const double waypoint_lat = mission_item["params"][4];
        const double waypoint_lon = mission_item["params"][5];
        const double waypoint_relative_alt = mission_item["params"][6];
        Eigen::Vector3d waypoint_lv03;
        GeoConversions::forward(waypoint_lat, waypoint_lon, 0.0, waypoint_lv03.x(), waypoint_lv03.y(),
                                waypoint_lv03.z());  /// FIXME: Assuming zero AMSL
        ESPG map_coordinate;
        Eigen::Vector3d map_origin;
        terrain_map->getGlobalOrigin(map_coordinate, map_origin);
        Eigen::Vector3d waypoint_local = waypoint_lv03 - map_origin;
        waypoint_local.z() =
            terrain_map->getGridMap().atPosition("elevation", waypoint_local.head(2)) + waypoint_relative_alt;
        waypoint_list.push_back(waypoint_local);
      }
    }
  }
  /// TODO: Generate Dubins path segments from waypoint list
  Path mission_path = generatePathFromWaypoints(waypoint_list, radius);

  /// Parse rally points from mission file
  std::vector<Eigen::Vector3d> rallypoint_list;
  const double map_width_x = terrain_map->getGridMap().getLength().x();
  const double map_width_y = terrain_map->getGridMap().getLength().y();
  const Eigen::Vector2d map_pos = terrain_map->getGridMap().getPosition();

  json mission_rallypoints = data["rallyPoints"]["points"];
  for (auto rallypoint : mission_rallypoints) {
    std::cout << "  - Rallypoint: " << rallypoint << std::endl;
    const double rallypoint_lat = rallypoint[0];
    const double rallypoint_lon = rallypoint[1];
    const double rallypoint_relative_alt = rallypoint[2];
    Eigen::Vector3d rallypoint_lv03;
    GeoConversions::forward(rallypoint_lat, rallypoint_lon, 0.0, rallypoint_lv03.x(), rallypoint_lv03.y(),
                            rallypoint_lv03.z());  /// FIXME: Assuming zero AMSL
    ESPG map_coordinate;
    Eigen::Vector3d map_origin;
    terrain_map->getGlobalOrigin(map_coordinate, map_origin);
    Eigen::Vector3d rallypoint_local = rallypoint_lv03 - map_origin;
    rallypoint_local.z() =
        terrain_map->getGridMap().atPosition("elevation", rallypoint_local.head(2)) + rallypoint_relative_alt;
    rallypoint_list.push_back(rallypoint_local);
  }

  /// TODO: Generate path from waypoint list

  // Initialize planner with loaded terrain map
  auto planner = std::make_shared<TerrainOmplRrt>();
  planner->setMap(terrain_map);
  /// TODO: Get bounds from gridmap
  planner->setBoundsFromMap(terrain_map->getGridMap());

  /// TODO: Iterate over path segments to find abort paths
  int segment_id = 0;
  Path abort_path_list;
  // for (const auto& path_segment : path.segments) {
  //   std::cout << "  - segment ID: " << segment_id << std::endl;
  //   Eigen::Vector3d segment_end = path_segment.states.back().position;
  //   Eigen::Vector3d segment_end_tangent = path_segment.states.back().velocity;
  //   planner->setupProblem(segment_end, segment_end_tangent, rallypoint_list);
  //   Path abort_path;
  //   if (planner->Solve(10.0, abort_path)) {
  //     std::cout << "[TestRRTCircleGoal] Found Solution!" << std::endl;
  //     abort_path_list.appendSegment(abort_path);
  //   } else {
  //     std::cout << "[TestRRTCircleGoal] Unable to find solution" << std::endl;
  //   }
  //   segment_id++;
  // }

  // Repeatedly publish results
  while (true) {
    terrain_map->getGridMap().setTimestamp(ros::Time::now().toNSec());
    publishGridMap(grid_map_pub, terrain_map->getGridMap());
    publishWaypoints(waypoint_pub, waypoint_list);
    publishRallyPoints(rallypoint_pub, rallypoint_list, radius, Eigen::Vector3d(1.0, 0.0, 0.0));
    publishGeoFence(geofence_pub, geofence_polygon);  /// Visualize Geofence

    publishPathSegments(abort_path_pub, abort_path_list, Eigen::Vector3d(1.0, 0.0, 0.0));
    publishPathSegments(mission_path_pub, mission_path);
    ros::Duration(1.0).sleep();
    ros::spinOnce();
  }

  return 0;
}
