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
#include "terrain_navigation_ros/visualization.h"
#include "terrain_planner/common.h"
#include "terrain_planner/terrain_ompl_rrt.h"
#include "terrain_planner_benchmark/visualization.h"

bool validatePosition(std::shared_ptr<TerrainMap> map, const Eigen::Vector3d goal, Eigen::Vector3d &valid_goal) {
  double upper_surface = map->getGridMap().atPosition("ics_+", goal.head(2));
  double lower_surface = map->getGridMap().atPosition("ics_-", goal.head(2));
  const bool is_goal_valid = (upper_surface < lower_surface) ? true : false;
  valid_goal(0) = goal(0);
  valid_goal(1) = goal(1);
  valid_goal(2) = (upper_surface + lower_surface) / 2.0;
  return is_goal_valid;
}

double mod2pi(double x) { return x - 2 * M_PI * floor(x * (0.5 / M_PI)); }

Eigen::Vector4d rpy2quaternion(double roll, double pitch, double yaw) {
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

std::vector<Eigen::Vector3d> sampleRallyPoints(const int num_rally_points, Eigen::Vector3d start_position,
                                               grid_map::GridMap &map) {
  std::vector<Eigen::Vector3d> rally_points;
  for (int i = 0; i < num_rally_points; i++) {
    bool sample_is_valid = false;
    while (!sample_is_valid) {
      Eigen::Vector3d random_sample;
      random_sample(0) = getRandom(-200.0, 200.0);
      random_sample(1) = getRandom(-200.0, 200.0);
      Eigen::Vector3d candidate_loiter_position = start_position + random_sample;
      Eigen::Vector3d new_loiter_position;
      sample_is_valid = validatePosition(map, candidate_loiter_position, new_loiter_position);
      if (sample_is_valid) {
        rally_points.push_back(new_loiter_position);
      }
    }
  }
  return rally_points;
}

visualization_msgs::Marker getGoalMarker(const int id, const Eigen::Vector3d &position, const double radius,
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

PathSegment getLoiterPath(Eigen::Vector3d end_position, Eigen::Vector3d end_velocity, Eigen::Vector3d center_pos) {
  Eigen::Vector3d radial_vector = (end_position - center_pos);
  radial_vector(2) = 0.0;  // Only consider horizontal loiters
  Eigen::Vector3d emergency_rates =
      20.0 * end_velocity.normalized().cross(radial_vector.normalized()) / radial_vector.norm();
  double horizon = 2 * M_PI / std::abs(emergency_rates(2));
  // Append a loiter at the end of the planned path
  PathSegment loiter_trajectory = generateArcTrajectory(emergency_rates, horizon, end_position, end_velocity);
  return loiter_trajectory;
}

void publishRallyPoints(const ros::Publisher &pub, const std::vector<Eigen::Vector3d> &positions, const double radius,
                        Eigen::Vector3d color, std::string name_space = "rallypoints") {
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

int main(int argc, char **argv) {
  ros::init(argc, argv, "ompl_rrt_planner");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  // Initialize ROS related publishers for visualization
  auto start_pos_pub = nh.advertise<visualization_msgs::Marker>("start_position", 1, true);
  auto goal_pos_pub = nh.advertise<visualization_msgs::Marker>("goal_position", 1, true);
  auto path_pub = nh.advertise<nav_msgs::Path>("path", 1, true);
  auto grid_map_pub = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  auto trajectory_pub = nh.advertise<visualization_msgs::MarkerArray>("tree", 1, true);
  auto path_segment_pub = nh.advertise<visualization_msgs::MarkerArray>("path_segments", 1, true);
  auto abort_path_segment_pub = nh.advertise<visualization_msgs::MarkerArray>("path_segments_abort", 1, true);
  auto rallypoint_pub = nh.advertise<visualization_msgs::MarkerArray>("rallypoints_marker", 1);

  std::string map_path, color_file_path, output_directory, location;
  nh_private.param<std::string>("map_path", map_path, "");
  nh_private.param<std::string>("color_file_path", color_file_path, "");
  nh_private.param<std::string>("location", location, "");
  nh_private.param<std::string>("output_directory", output_directory, "");

  // Initialize data logger for recording
  auto data_logger = std::make_shared<DataLogger>();
  data_logger->setKeys({"x", "y", "z"});

  // Load terrain map from defined tif paths
  auto terrain_map = std::make_shared<TerrainMap>();
  terrain_map->initializeFromGeotiff(map_path, false);
  if (!color_file_path.empty()) {  // Load color layer if the color path is nonempty
    terrain_map->addColorFromGeotiff(color_file_path);
  }
  terrain_map->AddLayerDistanceTransform(50.0, "distance_surface");
  terrain_map->AddLayerDistanceTransform(120.0, "max_elevation");
  double radius = 66.67;
  terrain_map->AddLayerHorizontalDistanceTransform(radius, "ics_+", "distance_surface");
  terrain_map->AddLayerHorizontalDistanceTransform(-radius, "ics_-", "max_elevation");
  terrain_map->addLayerSafety("safety", "ics_+", "ics_-");

  Path path;

  // Initialize planner with loaded terrain map
  auto planner = std::make_shared<TerrainOmplRrt>();
  planner->setMap(terrain_map);
  /// TODO: Get bounds from gridmap
  planner->setBoundsFromMap(terrain_map->getGridMap());

  const double map_width_x = terrain_map->getGridMap().getLength().x();
  const double map_width_y = terrain_map->getGridMap().getLength().y();
  const Eigen::Vector2d map_pos = terrain_map->getGridMap().getPosition();

  Eigen::Vector3d start{Eigen::Vector3d(map_pos(0) + 0.4 * map_width_x, map_pos(1) - 0.35 * map_width_y, 0.0)};
  Eigen::Vector3d updated_start;
  if (validatePosition(terrain_map, start, updated_start)) {
    start = updated_start;
    std::cout << "Specified start position is valid" << std::endl;
  } else {
    throw std::runtime_error("Specified start position is NOT valid");
  }
  Eigen::Vector3d goal{Eigen::Vector3d(map_pos(0) - 0.3 * map_width_x, map_pos(1) + 0.3 * map_width_y, 0.0)};
  Eigen::Vector3d updated_goal;
  if (validatePosition(terrain_map, goal, updated_goal)) {
    goal = updated_goal;
    std::cout << "Specified goal position is valid" << std::endl;
  } else {
    throw std::runtime_error("Specified goal position is NOT valid");
  }

  planner->setupProblem(start, goal);
  if (planner->Solve(15.0, path)) {
    std::cout << "[TestRRTCircleGoal] Found Solution!" << std::endl;
  } else {
    std::cout << "[TestRRTCircleGoal] Unable to find solution" << std::endl;
  }

  Eigen::Vector3d start_position = path.firstSegment().states.front().position;
  Eigen::Vector3d start_velocity = path.firstSegment().states.front().velocity;

  PathSegment start_loiter_path = getLoiterPath(start_position, start_velocity, start);
  path.prependSegment(start_loiter_path);

  Eigen::Vector3d end_position = path.lastSegment().states.back().position;
  Eigen::Vector3d end_velocity = path.lastSegment().states.back().velocity;
  PathSegment goal_loiter_path = getLoiterPath(end_position, end_velocity, goal);

  path.appendSegment(goal_loiter_path);

  Path path_abort;
  /// Sample rally points
  auto rally_points = sampleRallyPoints(3, start_position, terrain_map->getGridMap());
  /// TODO: Get end of some segment
  planner->setupProblem(start_position, start_velocity, rally_points);
  if (planner->Solve(15.0, path_abort)) {
    std::cout << "[TestRRTCircleGoal] Found Abort Solution!" << std::endl;
  }
  Eigen::Vector3d end_position_abort = path_abort.lastSegment().states.back().position;
  Eigen::Vector3d end_velocity_abort = path_abort.lastSegment().states.back().velocity;
  /// TODO: Figure out which rally point the planner is using
  double min_distance_error = std::numeric_limits<double>::infinity();
  int min_distance_index = -1;
  for (int idx = 0; idx < rally_points.size(); idx++) {
    double radial_error = std::abs((end_position - rally_points[idx]).norm() - radius);
    if (radial_error < min_distance_error) {
      min_distance_index = idx;
      min_distance_error = radial_error;
    }
  }
  // PathSegment rally_loiter_path = getLoiterPath(end_position_abort, end_velocity_abort,
  // rally_points[min_distance_index]);

  publishPathSegments(abort_path_segment_pub, path_abort);
  publishRallyPoints(rallypoint_pub, rally_points, 66.67, Eigen::Vector3d(1.0, 1.0, 0.0));

  // Repeatedly publish results
  terrain_map->getGridMap().setTimestamp(ros::Time::now().toNSec());
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(terrain_map->getGridMap(), message);
  grid_map_pub.publish(message);
  publishTrajectory(path_pub, path.position());

  /// TODO: Publish a circle instead of a goal marker!
  publishCircleSetpoints(start_pos_pub, start, radius);
  publishCircleSetpoints(goal_pos_pub, goal, radius);
  publishTree(trajectory_pub, planner->getPlannerData(), planner->getProblemSetup());
  publishPathSegments(path_segment_pub, path);
  /// Save planned path into a csv file for plotting
  for (auto &point : path.position()) {
    std::unordered_map<std::string, std::any> state;
    state.insert(std::pair<std::string, double>("x", point(0) + 0.5 * map_width_x));
    state.insert(std::pair<std::string, double>("y", point(1) + 0.5 * map_width_y));
    state.insert(std::pair<std::string, double>("z", point(2)));
    data_logger->record(state);
  }

  data_logger->setPrintHeader(true);
  std::string output_file_path = output_directory + "/" + location + "_planned_path.csv";
  data_logger->writeToFile(output_file_path);

  ros::Duration(1.0).sleep();
  ros::spin();
  return 0;
}
