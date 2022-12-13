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
 * @brief ROS Node to test ompl
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>
#include <chrono>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include "terrain_navigation/data_logger.h"
#include "terrain_planner/common.h"
#include "terrain_planner/terrain_ompl_rrt.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "ompl_rrt_planner");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  std::string file_path;
  nh_private.param<std::string>("file_path", file_path, "output/timings.csv");

  std::vector<double> timings;

  auto dubins_ss = std::make_shared<fw_planning::spaces::DubinsAirplaneStateSpace>();

  std::vector<std::string> keys({"exhaustive", "classified"});

  auto logger = std::make_shared<DataLogger>();
  logger->setPrintHeader(true);
  logger->setKeys(keys);

  int num_experiments{100};
  for (int idx = 0; idx < num_experiments; idx++) {
    /// TODO: Randomly generate states
    fw_planning::spaces::DubinsPath dp;
    double d{10.0};
    double alpha = getRandom(0, 2 * M_PI);
    double beta = getRandom(0, 2 * M_PI);
    std::unordered_map<std::string, std::any> state;
    for (auto& key : keys) {
      if (key == "exhaustive") {
        dubins_ss->setEnableSetClassification(false);
      } else {
        dubins_ss->setEnableSetClassification(true);
      }
      auto start = std::chrono::steady_clock::now();
      dubins_ss->dubins(d, alpha, beta, dp);
      if (!isfinite(dp.length_2D())) {
        std::cout << "Length is invalid!" << std::endl;
        std::cout << "  - key: " << key << std::endl;
      }
      auto end = std::chrono::steady_clock::now();
      double duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
      state.insert(std::pair<std::string, double>(key, duration));
    }
    logger->record(state);
  }
  logger->writeToFile(file_path);
  return 0;
}
