/****************************************************************************
 *
 *   Copyright (c) 2023 Jaeyoung Lim. All rights reserved.
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

#ifndef TERRAIN_NAVIGATION_ROS_VISUALIZATION_H
#define TERRAIN_NAVIGATION_ROS_VISUALIZATION_H

#include <ros/ros.h>
#include <Eigen/Dense>

#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "terrain_navigation/path.h"
#include "terrain_planner/common.h"
#include "terrain_planner/ompl_setup.h"

inline geometry_msgs::Point toPoint(const Eigen::Vector3d& p) {
  geometry_msgs::Point position;
  position.x = p(0);
  position.y = p(1);
  position.z = p(2);
  return position;
}

inline void publishVehiclePose(const ros::Publisher pub, const Eigen::Vector3d& position,
                               const Eigen::Vector4d& attitude, std::string mesh_resource_path) {
  Eigen::Vector4d mesh_attitude =
      quatMultiplication(attitude, Eigen::Vector4d(std::cos(M_PI / 2), 0.0, 0.0, std::sin(M_PI / 2)));
  geometry_msgs::Pose vehicle_pose = vector3d2PoseMsg(position, mesh_attitude);
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "map";
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.ns = "my_namespace";
  marker.mesh_resource = "package://terrain_planner/" + mesh_resource_path;
  marker.scale.x = 10.0;
  marker.scale.y = 10.0;
  marker.scale.z = 10.0;
  marker.color.a = 0.5;  // Don't forget to set the alpha!
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.pose = vehicle_pose;
  pub.publish(marker);
}

inline visualization_msgs::Marker Viewpoint2MarkerMsg(int id, ViewPoint& viewpoint,
                                                      Eigen::Vector3d color = Eigen::Vector3d(0.0, 0.0, 1.0)) {
  double scale{15};  // Size of the viewpoint markers
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  const Eigen::Vector3d position = viewpoint.getCenterLocal();
  std::vector<geometry_msgs::Point> points;
  std::vector<Eigen::Vector3d> corner_ray_vectors = viewpoint.getCornerRayVectors();
  std::vector<Eigen::Vector3d> vertex;
  for (auto& corner_ray : corner_ray_vectors) {
    vertex.push_back(position + scale * corner_ray);
  }

  for (size_t i = 0; i < vertex.size(); i++) {
    points.push_back(toPoint(position));  // Viewpoint center
    points.push_back(toPoint(vertex[i]));
    points.push_back(toPoint(vertex[i]));
    points.push_back(toPoint(vertex[(i + 1) % vertex.size()]));
  }

  marker.points = points;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.a = 1.0;  // Don't forget to set the alpha!
  marker.color.r = color(0);
  marker.color.g = color(1);
  marker.color.b = color(2);
  return marker;
}

inline void publishCameraView(const ros::Publisher pub, const Eigen::Vector3d& position,
                              const Eigen::Vector4d& attitude) {
  visualization_msgs::Marker marker;
  ViewPoint viewpoint(-1, position, attitude);
  marker = Viewpoint2MarkerMsg(viewpoint.getIndex(), viewpoint);
  pub.publish(marker);
}

inline void publishViewpoints(const ros::Publisher pub, std::vector<ViewPoint>& viewpoint_vector,
                              Eigen::Vector3d color = Eigen::Vector3d(0.0, 0.0, 1.0)) {
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
    viewpoint_marker_vector.insert(viewpoint_marker_vector.begin(), Viewpoint2MarkerMsg(i, viewpoint, color));
    i++;
  }
  msg.markers = viewpoint_marker_vector;
  pub.publish(msg);
}

void publishCandidateManeuvers(const ros::Publisher& pub, const std::vector<Path>& candidate_maneuvers,
                               bool visualize_invalid_trajectories = false) {
  visualization_msgs::MarkerArray msg;

  std::vector<visualization_msgs::Marker> marker;
  visualization_msgs::Marker mark;
  mark.action = visualization_msgs::Marker::DELETEALL;
  marker.push_back(mark);
  msg.markers = marker;
  pub.publish(msg);

  std::vector<visualization_msgs::Marker> maneuver_library_vector;
  int i = 0;
  for (auto maneuver : candidate_maneuvers) {
    if (maneuver.valid() || visualize_invalid_trajectories) {
      maneuver_library_vector.insert(maneuver_library_vector.begin(), trajectory2MarkerMsg(maneuver, i));
    }
    i++;
  }
  msg.markers = maneuver_library_vector;
  pub.publish(msg);
}

void publishPositionSetpoints(const ros::Publisher& pub, const Eigen::Vector3d& position,
                              const Eigen::Vector3d& velocity,
                              const Eigen::Vector3d scale = Eigen::Vector3d(10.0, 2.0, 2.0)) {
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::ARROW;
  marker.header.frame_id = "map";
  marker.id = 0;
  marker.action = visualization_msgs::Marker::DELETEALL;
  pub.publish(marker);

  marker.header.stamp = ros::Time::now();
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = scale(0);
  marker.scale.y = scale(1);
  marker.scale.z = scale(2);
  marker.color.a = 0.5;  // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.pose.position = tf2::toMsg(position);
  tf2::Quaternion q;
  q.setRPY(0, 0, std::atan2(velocity.y(), velocity.x()));
  marker.pose.orientation = tf2::toMsg(q);

  pub.publish(marker);
}

void publishPath(const ros::Publisher& pub, std::vector<Eigen::Vector3d> path, Eigen::Vector3d color) {
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.header.frame_id = "map";
  marker.id = 0;
  marker.action = visualization_msgs::Marker::ADD;
  std::vector<geometry_msgs::Point> points;
  for (auto& position : path) {
    geometry_msgs::Point point;
    point.x = position(0);
    point.y = position(1);
    point.z = position(2);
    points.push_back(point);
  }
  std::cout << "Points: " << points.size() << std::endl;
  marker.points = points;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 10.0;
  marker.scale.y = 10.0;
  marker.scale.z = 10.0;
  marker.color.a = 0.8;
  marker.color.r = color.x();
  marker.color.g = color.y();
  marker.color.b = color.z();
  pub.publish(marker);
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

void publishTree(const ros::Publisher& pub, std::shared_ptr<ompl::base::PlannerData> planner_data,
                 std::shared_ptr<ompl::OmplSetup> problem_setup) {
  visualization_msgs::MarkerArray marker_array;
  std::vector<visualization_msgs::Marker> marker;

  planner_data->decoupleFromPlanner();

  // Create states, a marker and a list to store edges
  ompl::base::ScopedState<fw_planning::spaces::DubinsAirplaneStateSpace> vertex(problem_setup->getSpaceInformation());
  ompl::base::ScopedState<fw_planning::spaces::DubinsAirplaneStateSpace> neighbor_vertex(
      problem_setup->getSpaceInformation());
  size_t marker_idx{0};
  auto dubins_ss = std::make_shared<fw_planning::spaces::DubinsAirplaneStateSpace>();
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
    marker.pose.orientation.w = std::cos(0.5 * vertex[3]);
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = std::sin(0.5 * vertex[3]);
    marker.scale.x = 10.0;
    marker.scale.y = 2.0;
    marker.scale.z = 2.0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = 0.5;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker_array.markers.push_back(marker);

    // allocate variables
    std::vector<unsigned int> edge_list;
    int num_edges = planner_data->getEdges(i, edge_list);
    if (num_edges > 0) {
      for (unsigned int edge : edge_list) {
        visualization_msgs::Marker edge_marker;
        edge_marker.header.stamp = ros::Time().now();
        edge_marker.header.frame_id = "map";
        edge_marker.id = marker_idx++;
        edge_marker.type = visualization_msgs::Marker::LINE_STRIP;
        edge_marker.ns = "edge";
        neighbor_vertex = planner_data->getVertex(edge).getState();
        // points.push_back(toMsg(Eigen::Vector3d(vertex[0], vertex[1], vertex[2])));
        // points.push_back(toMsg(Eigen::Vector3d(neighbor_vertex[0], neighbor_vertex[1], neighbor_vertex[2])));
        ompl::base::State* state = dubins_ss->allocState();
        ompl::base::State* from = dubins_ss->allocState();
        from->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setX(vertex[0]);
        from->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setY(vertex[1]);
        from->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setZ(vertex[2]);
        from->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setYaw(vertex[3]);

        ompl::base::State* to = dubins_ss->allocState();
        to->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setX(neighbor_vertex[0]);
        to->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setY(neighbor_vertex[1]);
        to->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setZ(neighbor_vertex[2]);
        to->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->setYaw(neighbor_vertex[3]);
        if (dubins_ss->equalStates(from, to)) {
          continue;
        }
        std::vector<geometry_msgs::Point> points;
        for (double t = 0.0; t < 1.0; t += 0.02) {
          dubins_ss->interpolate(from, to, t, state);
          auto interpolated_state =
              Eigen::Vector3d(state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getX(),
                              state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getY(),
                              state->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>()->getZ());
          points.push_back(tf2::toMsg(interpolated_state));
        }
        points.push_back(tf2::toMsg(Eigen::Vector3d(neighbor_vertex[0], neighbor_vertex[1], neighbor_vertex[2])));
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
  }
  pub.publish(marker_array);
}

#endif
