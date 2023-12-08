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

#include <terrain_navigation/terrain_map.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <functional>
#include <geometry_msgs/msg/point.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "terrain_planner/common.h"
#include "terrain_planner/terrain_ompl_rrt.h"
#include "terrain_planner/visualization.h"

//! @todo(srmainwaring)check includes
// #include "terrain_planner/common.h"
// #include "terrain_planner/DubinsAirplane.hpp"
// #include "terrain_planner/DubinsPath.hpp"
// #include "terrain_planner/maneuver_library.h"
// #include "terrain_planner/ompl_setup.h"
// #include "terrain_planner/planner.h"
// #include "terrain_planner/terrain_ompl_rrt.h"
// #include "terrain_planner/terrain_ompl.h"
#include "terrain_planner/visualization.h"

using namespace std::chrono_literals;

void publishPathSegments(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub, Path& trajectory) {
  visualization_msgs::msg::MarkerArray msg;

  std::vector<visualization_msgs::msg::Marker> marker;
  visualization_msgs::msg::Marker mark;
  mark.action = visualization_msgs::msg::Marker::DELETEALL;
  marker.push_back(mark);
  msg.markers = marker;
  pub->publish(msg);

  std::vector<visualization_msgs::msg::Marker> segment_markers;
  int i = 0;
  for (auto& segment : trajectory.segments) {
    Eigen::Vector3d color = Eigen::Vector3d(1.0, 0.0, 0.0);
    if (segment.curvature > 0.0) {  // Green is DUBINS_LEFT
      color = Eigen::Vector3d(0.0, 1.0, 0.0);
    } else if (segment.curvature < 0.0) {  // Blue is DUBINS_RIGHT
      color = Eigen::Vector3d(0.0, 0.0, 1.0);
    }
    segment_markers.insert(segment_markers.begin(), trajectory2MarkerMsg(segment, i++, color));
    segment_markers.insert(segment_markers.begin(),
                           vector2ArrowsMsg(segment.position().front(), 5.0 * segment.velocity().front(), i++, color));
    segment_markers.insert(segment_markers.begin(), point2MarkerMsg(segment.position().back(), i++, color));
  }
  msg.markers = segment_markers;
  pub->publish(msg);
}

class OmplRrtPlanner : public rclcpp::Node {
 public:
  OmplRrtPlanner() : Node("ompl_rrt_planner") {
    // Initialize ROS related publishers for visualization
    start_pos_pub = this->create_publisher<visualization_msgs::msg::Marker>("start_position", 1);
    goal_pos_pub = this->create_publisher<visualization_msgs::msg::Marker>("goal_position", 1);
    path_pub = this->create_publisher<nav_msgs::msg::Path>("path", 1);
    interpolate_path_pub = this->create_publisher<nav_msgs::msg::Path>("interpolated_path", 1);
    path_segment_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("path_segments", 1);
    grid_map_pub = this->create_publisher<grid_map_msgs::msg::GridMap>("grid_map", 1);
    trajectory_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("tree", 1);

    map_path = this->declare_parameter("map_path", ".");
    color_file_path = this->declare_parameter("color_file_path", ".");
    random = this->declare_parameter("random", random);

    RCLCPP_INFO_STREAM(get_logger(), "map_path: " << map_path);
    RCLCPP_INFO_STREAM(get_logger(), "color_file_path: " << color_file_path);
    RCLCPP_INFO_STREAM(get_logger(), "random: " << random);

    // Load terrain map from defined tif paths
    terrain_map = std::make_shared<TerrainMap>();
    terrain_map->initializeFromGeotiff(map_path, false);
    // Load color layer if the color path is nonempty
    if (!color_file_path.empty()) {
      terrain_map->addColorFromGeotiff(color_file_path);
    }
    terrain_map->AddLayerDistanceTransform(50.0, "distance_surface");
    terrain_map->AddLayerDistanceTransform(120.0, "max_elevation");

    timer = this->create_wall_timer(1s, std::bind(&OmplRrtPlanner::timer_callback, this));
  }

  void timer_callback() {
    // for (int i = 0; i < num_experiments; i++) {
    if (count_experiments++ < num_experiments) {
      // Initialize planner with loaded terrain map
      auto planner = std::make_shared<TerrainOmplRrt>();
      planner->setMap(terrain_map);
      planner->setAltitudeLimits(120.0, 50.0);
      /// TODO: Get bounds from gridmap
      planner->setBoundsFromMap(terrain_map->getGridMap());

      const Eigen::Vector2d map_pos = terrain_map->getGridMap().getPosition();
      const double map_width_x = terrain_map->getGridMap().getLength().x();
      const double map_width_y = terrain_map->getGridMap().getLength().y();

      Eigen::Vector3d start{Eigen::Vector3d(map_pos(0) - 0.4 * map_width_x, map_pos(1) - 0.4 * map_width_y, 0.0)};
      start(2) =
          terrain_map->getGridMap().atPosition("elevation", Eigen::Vector2d(start(0), start(1))) + terrain_altitude;
      double start_yaw = getRandom(-M_PI, M_PI);
      Eigen::Vector3d start_vel = 10.0 * Eigen::Vector3d(std::cos(start_yaw), std::sin(start_yaw), 0.0);
      Eigen::Vector3d goal{Eigen::Vector3d(map_pos(0) + 0.4 * map_width_x, map_pos(1) + 0.4 * map_width_y, 0.0)};
      goal(2) = terrain_map->getGridMap().atPosition("elevation", Eigen::Vector2d(goal(0), goal(1))) + terrain_altitude;
      double goal_yaw = getRandom(-M_PI, M_PI);
      Eigen::Vector3d goal_vel = 10.0 * Eigen::Vector3d(std::cos(goal_yaw), std::sin(goal_yaw), 0.0);

      planner->setupProblem(start, start_vel, goal, goal_vel);
      bool found_solution{false};
      while (!found_solution) {
        found_solution = planner->Solve(1.0, path);
      }
      planner->getSolutionPath(interpolated_path);

      // Repeatedly publish results
      terrain_map->getGridMap().setTimestamp(this->get_clock()->now().nanoseconds());
      auto message = grid_map::GridMapRosConverter::toMessage(terrain_map->getGridMap());
      grid_map_pub->publish(std::move(message));
      publishTrajectory(path_pub, path.position());
      publishTrajectory(interpolate_path_pub, interpolated_path);
      publishPathSegments(path_segment_pub, path);
      publishPositionSetpoints(start_pos_pub, start, start_vel);
      publishPositionSetpoints(goal_pos_pub, goal, goal_vel);
      publishTree(trajectory_pub, planner->getPlannerData(), planner->getProblemSetup());
      if (!random) {
        count_experiments = num_experiments;
        // break;
      }
      // rclcpp::sleep_for(1s);
    }
  }

 private:
  std::string map_path;
  std::string color_file_path;
  bool random{false};

  std::shared_ptr<TerrainMap> terrain_map;
  Path path;
  std::vector<Eigen::Vector3d> interpolated_path;
  double terrain_altitude{100.0};

  int num_experiments{20};
  int count_experiments{0};

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr start_pos_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_pos_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr interpolate_path_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_segment_pub;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_pub;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto ompl_rrt_planner = std::make_shared<OmplRrtPlanner>();
  rclcpp::spin(ompl_rrt_planner);
  rclcpp::shutdown();
  return 0;
}
