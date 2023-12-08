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

#include "terrain_navigation_ros/visualization.h"

#include <terrain_planner/common.h>

geometry_msgs::msg::Point toPoint(const Eigen::Vector3d &p) {
  geometry_msgs::msg::Point position;
  position.x = p(0);
  position.y = p(1);
  position.z = p(2);
  return position;
}

void publishVehiclePose(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                        const Eigen::Vector3d &position, const Eigen::Vector4d &attitude,
                        std::string mesh_resource_path) {
  Eigen::Vector4d mesh_attitude =
      quatMultiplication(attitude, Eigen::Vector4d(std::cos(M_PI / 2), 0.0, 0.0, std::sin(M_PI / 2)));
  geometry_msgs::msg::Pose vehicle_pose = vector3d2PoseMsg(position, mesh_attitude);
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = rclcpp::Clock().now();
  marker.header.frame_id = "map";
  marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  marker.ns = "my_namespace";
  //! @todo(srmainwaring) understand why the mesh is not displayed.
  //! @note https://answers.ros.org/question/282745/rviz-doesnt-load-dae-mesh-cannot-locate-it/
  // marker.mesh_resource = "package://terrain_planner/" + mesh_resource_path;
  marker.mesh_resource = "file://" + mesh_resource_path;
  marker.scale.x = 10.0;
  marker.scale.y = 10.0;
  marker.scale.z = 10.0;
  marker.color.a = 0.5;  // Don't forget to set the alpha!
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.pose = vehicle_pose;
  pub->publish(marker);
}

visualization_msgs::msg::Marker Viewpoint2MarkerMsg(int id, ViewPoint &viewpoint, Eigen::Vector3d color) {
  double scale{15};  // Size of the viewpoint markers
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = rclcpp::Clock().now();
  marker.ns = "my_namespace";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  const Eigen::Vector3d position = viewpoint.getCenterLocal();
  std::vector<geometry_msgs::msg::Point> points;
  std::vector<Eigen::Vector3d> corner_ray_vectors = viewpoint.getCornerRayVectors();
  std::vector<Eigen::Vector3d> vertex;
  for (auto &corner_ray : corner_ray_vectors) {
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

void publishCameraView(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                       const Eigen::Vector3d &position, const Eigen::Vector4d &attitude) {
  visualization_msgs::msg::Marker marker;
  ViewPoint viewpoint(-1, position, attitude);
  marker = Viewpoint2MarkerMsg(viewpoint.getIndex(), viewpoint);
  pub->publish(marker);
}

void publishViewpoints(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub,
                       std::vector<ViewPoint> &viewpoint_vector, Eigen::Vector3d color) {
  visualization_msgs::msg::MarkerArray msg;

  std::vector<visualization_msgs::msg::Marker> marker;
  visualization_msgs::msg::Marker mark;
  mark.action = visualization_msgs::msg::Marker::DELETEALL;
  marker.push_back(mark);
  msg.markers = marker;
  pub->publish(msg);

  std::vector<visualization_msgs::msg::Marker> viewpoint_marker_vector;
  int i = 0;
  for (auto viewpoint : viewpoint_vector) {
    viewpoint_marker_vector.insert(viewpoint_marker_vector.begin(), Viewpoint2MarkerMsg(i, viewpoint, color));
    i++;
  }
  msg.markers = viewpoint_marker_vector;
  pub->publish(msg);
}
