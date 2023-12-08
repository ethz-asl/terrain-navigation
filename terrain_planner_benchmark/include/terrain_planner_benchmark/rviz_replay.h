/****************************************************************************
 *
 *   Copyright (c) 2022 Jaeyoung Lim, Autonomous Systems Lab,
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
 * @brief Node to test planner in the view utiltiy map
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */
#include <grid_map_msgs/GridMap.h>
#include <planner_msgs/NavigationStatus.h>
#include <planner_msgs/TerrainInfo.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <grid_map_ros/GridMapRosConverter.hpp>

#include "terrain_planner/common.h"

class ReplayRunner {
 public:
  ReplayRunner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) : nh_(nh), nh_private_(nh_private) {
    grid_map_sub_ =
        nh_.subscribe("/grid_map", 1, &ReplayRunner::GridmapCallback, this, ros::TransportHints().tcpNoDelay());
    vehicle_position_sub_ = nh_.subscribe("/planner_status", 10, &ReplayRunner::plannerStatusCallback, this);
    planner_status_pub_ = nh_.advertise<planner_msgs::NavigationStatus>("planner_status2", 1);
    path_segment_pub_ = nh_.advertise<visualization_msgs::Marker>("visualized_path", 1);
    reference_visual_pub_ = nh_.advertise<visualization_msgs::Marker>("visualized_reference", 1);
    terrain_info_pub_ = nh_.advertise<planner_msgs::TerrainInfo>("terrain_info", 1);
    double statusloop_dt_ = 0.05;
    ros::TimerOptions statuslooptimer_options(
        ros::Duration(statusloop_dt_), boost::bind(&ReplayRunner::statusloopCallback, this, _1), &statusloop_queue_);
    statusloop_timer_ = nh_.createTimer(statuslooptimer_options);  // Define timer for constant loop rate

    statusloop_spinner_.reset(new ros::AsyncSpinner(1, &statusloop_queue_));
    statusloop_spinner_->start();
  };
  ~ReplayRunner(){};

 private:
  void GridmapCallback(const grid_map_msgs::GridMap& msg) {
    grid_map::GridMapRosConverter::fromMessage(msg, map_);
    initialized_map = true;
  };

  void plannerStatusCallback(const planner_msgs::NavigationStatus& msg) {
    vehicle_position_ = Eigen::Vector3d(msg.vehicle_position.x, msg.vehicle_position.y, msg.vehicle_position.z);
    reference_position_ = Eigen::Vector3d(msg.reference_position.x, msg.reference_position.y, msg.reference_position.z);
    timestamp_ = msg.header.stamp;
    // std::cout << "[RvizReplay] Receiving vehicle position: " << vehicle_position.transpose() << std::endl;
    updated_reference_ = true;
  };

  void statusloopCallback(const ros::TimerEvent& event) {
    // Increment segment ID depending on position
    // Get Map data
    if (initialized_map && updated_reference_) {
      vehicle_history_.push_back(vehicle_position_);
      reference_history_.push_back(reference_position_);

      Eigen::Vector3d reference_position = reference_history_.back();
      Eigen::Vector3d vehicle_position = vehicle_history_.back();

      bool is_segment_periodic =
          bool(std::abs(reference_position(2) - prev_reference_position_(2)) < 0.01);  // get current segment
      prev_reference_position_ = reference_position;
      if (is_segment_periodic != prev_segment_is_periodic_) {
        segment_id_++;
        prev_segment_is_periodic_ = is_segment_periodic;
      }
      segment_history_.push_back(segment_id_);

      double min_distance = std::numeric_limits<double>::infinity();
      for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) {
        Eigen::Vector3d terrain_surface;
        const grid_map::Index index = *iterator;
        map_.getPosition3("elevation", index, terrain_surface);
        double distance = (terrain_surface - vehicle_position).norm();
        if (distance < min_distance) {  // Update minimum distance
          min_distance = distance;
        }
      }

      planner_msgs::TerrainInfo terrain_info_msg;
      terrain_info_msg.header.stamp = timestamp_;
      terrain_info_msg.segment_id.data = segment_id_;
      terrain_info_msg.reference_position = toVector3(reference_position);
      terrain_info_msg.vehicle_position = toVector3(vehicle_position);
      terrain_info_msg.reference_terrain_altitude.data = map_.atPosition("elevation", reference_position.head(2));
      terrain_info_msg.reference_maximum_altitude.data = map_.atPosition("max_elevation", reference_position.head(2));
      terrain_info_msg.reference_minimum_altitude.data =
          map_.atPosition("distance_surface", reference_position.head(2));
      terrain_info_msg.reference_safety_lower_altitude.data = map_.atPosition("ics_-", reference_position.head(2));
      terrain_info_msg.reference_safety_upper_altitude.data = map_.atPosition("ics_+", reference_position.head(2));
      terrain_info_msg.vehicle_terrain_altitude.data = map_.atPosition("elevation", vehicle_position.head(2));
      terrain_info_msg.vehicle_maximum_altitude.data = map_.atPosition("max_elevation", vehicle_position.head(2));
      terrain_info_msg.vehicle_minimum_altitude.data = map_.atPosition("distance_surface", vehicle_position.head(2));
      terrain_info_msg.distance_to_terrain.data = min_distance;
      terrain_info_pub_.publish(terrain_info_msg);

      updated_reference_ = false;
    }

    // Visualize vehicle path and path reference
    visualization_msgs::Marker marker = trajectory2MarkerMsg(vehicle_history_, 0, Eigen::Vector3d(1.0, 0.0, 1.0));
    path_segment_pub_.publish(marker);

    // visualization_msgs::Marker reference_marker = trajectory2MarkerMsg(reference_history_, 0, "viridis");
    std::vector<Eigen::Vector3d> segment_colors;
    for (auto& segment_id : segment_history_) {
      const std::vector<std::vector<float>>& ctable = colorMap.at("gist_rainbow");
      double intensity = (segment_id - 2) / 8.0;
      // std::cout << "intensity: " << intensity << " segment_id: " << segment_id << std::endl;
      intensity = std::min(intensity, 1.0);
      intensity = std::max(intensity, 0.0);
      int idx = int(floor(intensity * 255.0));
      idx = std::min(idx, 255);
      idx = std::max(idx, 0);

      // Get color from table
      std::vector<float> rgb = ctable.at(idx);
      Eigen::Vector3d segment_color(static_cast<double>(rgb[0]), static_cast<double>(rgb[1]),
                                    static_cast<double>(rgb[2]));
      segment_colors.push_back(segment_color);
    }
    visualization_msgs::Marker reference_marker = trajectory2MarkerMsg(reference_history_, 0, segment_colors);
    reference_visual_pub_.publish(reference_marker);
  };

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber grid_map_sub_;
  ros::Publisher planner_status_pub_;
  ros::Publisher path_segment_pub_;
  ros::Publisher reference_visual_pub_;
  ros::Publisher terrain_info_pub_;
  ros::Subscriber vehicle_position_sub_;

  ros::Timer statusloop_timer_;
  ros::CallbackQueue statusloop_queue_;
  std::unique_ptr<ros::AsyncSpinner> statusloop_spinner_;

  std::vector<Eigen::Vector3d> vehicle_history_;
  std::vector<Eigen::Vector3d> reference_history_;
  std::vector<int> segment_history_;
  Eigen::Vector3d vehicle_position_;
  Eigen::Vector3d reference_position_;
  Eigen::Vector3d prev_reference_position_;

  int segment_id_{0};
  ros::Time timestamp_{0};

  bool initialized_map{false};
  bool prev_segment_is_periodic_{true};  // Normally we start with a loiter
  bool updated_reference_{false};
  grid_map::GridMap map_;
};
