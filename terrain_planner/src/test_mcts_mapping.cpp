/****************************************************************************
 *
 *   Copyright (c) 2022 Jaeyoung Lim. All rights reserved.
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
 * @brief ROS Node to test dubins planner
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include <ros/ros.h>
#include <terrain_planner/common.h>
#include "terrain_planner/mcts_planner.h"
#include "terrain_planner/visualization.h"

#include <terrain_navigation/profiler.h>
#include <terrain_navigation/visualization.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

void publishCandidateManeuvers(ros::Publisher &pub, const std::vector<TrajectorySegments> &candidate_maneuvers) {
  visualization_msgs::MarkerArray msg;

  std::vector<visualization_msgs::Marker> marker;
  visualization_msgs::Marker mark;
  mark.action = visualization_msgs::Marker::DELETEALL;
  marker.push_back(mark);
  msg.markers = marker;
  pub.publish(msg);

  std::vector<visualization_msgs::Marker> maneuver_library_vector;
  int i = 0;
  bool visualize_invalid_trajectories = false;
  for (auto maneuver : candidate_maneuvers) {
    if (maneuver.valid() || visualize_invalid_trajectories) {
      maneuver_library_vector.insert(maneuver_library_vector.begin(), trajectory2MarkerMsg(maneuver, i));
    }
    i++;
  }
  msg.markers = maneuver_library_vector;
  pub.publish(msg);
}

void publishViewpoints(ros::Publisher pub, std::vector<ViewPoint> &viewpoint_vector) {
  visualization_msgs::MarkerArray msg;

  std::vector<visualization_msgs::Marker> marker;
  visualization_msgs::Marker mark;
  mark.action = visualization_msgs::Marker::DELETEALL;
  marker.push_back(mark);
  msg.markers = marker;
  pub.publish(msg);

  std::vector<visualization_msgs::Marker> viewpoint_marker_vector;
  int i = 0;
  for (auto viewpoint : viewpoint_vector) {
    viewpoint_marker_vector.insert(viewpoint_marker_vector.begin(), Viewpoint2MarkerMsg(i, viewpoint));
    i++;
  }
  msg.markers = viewpoint_marker_vector;
  pub.publish(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dubins_planner");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  ros::Publisher start_pos_pub = nh.advertise<visualization_msgs::Marker>("start_position", 1, true);
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 1, true);
  ros::Publisher tree_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1, true);
  ros::Publisher gridmap_pub = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  ros::Publisher viewpoint_pub_ = nh.advertise<visualization_msgs::MarkerArray>("viewpoints", 1, true);

  std::string map_path, map_color_path;
  nh_private.param<std::string>("terrain_path", map_path, "resources/cadastre.tif");
  nh_private.param<std::string>("terrain_color_path", map_color_path, "");

  auto planner = std::make_shared<MctsPlanner>();

  auto terrain_map = std::make_shared<TerrainMap>();
  terrain_map->Load(map_path, false, map_color_path);

  auto viewutility_map = std::make_shared<ViewUtilityMap>(terrain_map->getGridMap());
  viewutility_map->initializeFromGridmap();
  planner->setViewUtilityMap(viewutility_map);

  Eigen::Vector3d start_pos{Eigen::Vector3d(0.0, 0.0, 0.0)};
  Eigen::Vector3d start_vel{Eigen::Vector3d(15.0, 0.0, 0.0)};
  Eigen::Vector4d start_att{Eigen::Vector4d(1.0, 0.0, 0.0, 0.0)};
  Eigen::Vector3d position = start_pos;
  Eigen::Vector3d velocity = start_vel;
  Eigen::Vector4d attitude = start_att;
  std::vector<ViewPoint> viewpoint_list;

  auto planner_profiler = std::make_shared<Profiler>("planner");
  auto mapper_profiler = std::make_shared<Profiler>("mapper");

  while (true) {
    // Run a single rollout of MCTS and visualize
    std::cout << "[TestMCTSMapping]  Run a single rollout of MCTS and visualize" << std::endl;
    planner_profiler->tic();
    TrajectorySegments current_path;
    TrajectorySegments path = planner->solve(position, velocity, attitude, current_path);
    std::vector<ViewPoint> candidate_viewpoint_list =
        ManeuverLibrary::sampleViewPointFromTrajectory(path.getCurrentSegment(position));
    std::cout << "[TestMCTSMapping]    - Planner Profiler: " << planner_profiler->toc() << std::endl;

    mapper_profiler->tic();
    double utility = viewutility_map->CalculateViewUtility(viewpoint_list, true);
    std::cout << "[TestMCTSMapping]    - Mapper Profiler: " << mapper_profiler->toc() << std::endl;

    viewpoint_list.insert(viewpoint_list.end(), candidate_viewpoint_list.begin(), candidate_viewpoint_list.end());
    publishPositionSetpoints(start_pos_pub, position, velocity);
    publishTrajectory(path_pub, path.position());
    publishCandidateManeuvers(tree_pub, planner->getMotionPrimitives());
    publishViewpoints(viewpoint_pub_, viewpoint_list);

    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(terrain_map->getGridMap(), message);
    gridmap_pub.publish(message);

    // Update states to the end of the current segment
    current_path = path;
    position = path.getCurrentSegment(position).position().back();
    velocity = path.getCurrentSegment(position).velocity().back();
    attitude = path.getCurrentSegment(position).attitude().back();
    ros::Duration(1.0).sleep();
  }

  ros::spin();
  return 0;
}
