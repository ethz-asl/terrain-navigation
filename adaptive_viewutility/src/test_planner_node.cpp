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
 * 3. Neither the name PX4 nor the names of its contributors may be
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
 * @brief Node to test planner in the view utiltiy map
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include "adaptive_viewutility/adaptive_viewutility.h"
#include "adaptive_viewutility/data_logger.h"
#include "adaptive_viewutility/performance_tracker.h"
#include "terrain_navigation/profiler.h"

void writeGridmapToImage(const grid_map::GridMap &map, const std::string layer, const std::string &file_path) {
  cv_bridge::CvImage image;
  grid_map::GridMapRosConverter::toCvImage(map, layer, sensor_msgs::image_encodings::MONO8, image);
  if (!cv::imwrite(file_path.c_str(), image.image, {cv::IMWRITE_PNG_STRATEGY_DEFAULT})) {
    std::cout << "Failed to write map to image: " << file_path << std::endl;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "adaptive_viewutility");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  std::string file_path, color_file_path, result_directory;
  int num_experiments;
  double max_experiment_duration;
  nh_private.param<std::string>("file_path", file_path, "");
  nh_private.param<std::string>("color_file_path", color_file_path, "");
  nh_private.param<int>("num_experiments", num_experiments, 1);
  nh_private.param<double>("max_experiment_duration", max_experiment_duration, 500);
  nh_private.param<std::string>("result_directory", result_directory, "output");

  ros::Publisher camera_path_pub = nh.advertise<nav_msgs::Path>("camera_path", 1, true);
  ros::Publisher camera_pose_pub = nh.advertise<geometry_msgs::PoseArray>("camera_poses", 1, true);
  ros::Publisher viewpoint_pub = nh.advertise<visualization_msgs::MarkerArray>("viewpoints", 1, true);
  /// set Current state of vehicle
  /// TODO: Randomly generate initial position
  std::vector<std::shared_ptr<DataLogger>> benchmark_results;

  for (int i = 0; i < num_experiments; i++) {
    std::cout << "Starting Experiment: " << i << std::endl;
    std::shared_ptr<AdaptiveViewUtility> adaptive_viewutility = std::make_shared<AdaptiveViewUtility>(nh, nh_private);

    // Add elevation map from GeoTIFF file defined in path
    adaptive_viewutility->LoadMap(file_path, color_file_path);

    grid_map::Polygon polygon;

    // Hah! we have access to the map reference. be careful!
    grid_map::GridMap &map = adaptive_viewutility->getViewUtilityMap()->getGridMap();
    const Eigen::Vector2d map_pos = map.getPosition();
    const double map_width_x = map.getLength().x();
    const double map_width_y = map.getLength().y();
    const double roi_portion = 0.2;

    polygon.setFrameId(map.getFrameId());
    polygon.addVertex(grid_map::Position(map_pos(0) - 0.5 * map_width_x, map_pos(1) - 0.5 * map_width_y));
    polygon.addVertex(grid_map::Position(map_pos(0) + 0.5 * map_width_x, map_pos(1) - 0.5 * map_width_y));
    polygon.addVertex(grid_map::Position(map_pos(0) + 0.5 * map_width_x, map_pos(1) + 0.5 * map_width_y));
    polygon.addVertex(grid_map::Position(map_pos(0) - 0.5 * map_width_x, map_pos(1) + 0.5 * map_width_y));

    adaptive_viewutility->getViewUtilityMap()->SetRegionOfInterest(polygon);

    Eigen::Vector3d vehicle_pos(map_pos(0), map_pos(1), 0.0);
    Eigen::Vector3d vehicle_vel(15.0, 0.0, 0.0);
    adaptive_viewutility->InitializeVehicleFromMap(vehicle_pos, vehicle_vel);
    std::cout << "  - Initial Position: " << vehicle_pos.transpose() << std::endl;
    std::cout << "  - Initial Velocity: " << vehicle_vel.transpose() << std::endl;

    adaptive_viewutility->setCurrentState(vehicle_pos, vehicle_vel);
    Profiler pipeline_perf("Planner Loop");
    std::shared_ptr<PerformanceTracker> performance_tracker = std::make_shared<PerformanceTracker>();

    bool terminate_mapping = false;
    double simulated_time{0.0};

    auto data_logger = std::make_shared<DataLogger>();
    data_logger->setKeys({"timestamp", "coverage", "quality"});

    while (true) {
      pipeline_perf.tic();
      adaptive_viewutility->generateMotionPrimitives();
      adaptive_viewutility->estimateViewUtility();

      Trajectory reference_trajectory = adaptive_viewutility->getBestPrimitive();

      Trajectory first_segment;
      for (int i = 0; i < 5; i++) {
        first_segment.states.push_back(reference_trajectory.states[i]);
      }

      adaptive_viewutility->UpdateUtility(first_segment);

      adaptive_viewutility->setCurrentState(first_segment.states.back().position, first_segment.states.back().velocity);

      adaptive_viewutility->MapPublishOnce();
      adaptive_viewutility->ViewpointPublishOnce(camera_path_pub, camera_pose_pub);
      adaptive_viewutility->publishViewpoint(viewpoint_pub);
      pipeline_perf.toc();

      double planning_horizon = adaptive_viewutility->getViewPlanner()->getPlanningHorizon();
      simulated_time += planning_horizon;

      Metrics performance =
          performance_tracker->Record(simulated_time, adaptive_viewutility->getViewUtilityMap()->getGridMap());

      std::unordered_map<std::string, std::any> state;
      state.insert(std::pair<std::string, double>("timestamp", performance.time));
      state.insert(std::pair<std::string, double>("coverage", performance.coverage));
      state.insert(std::pair<std::string, double>("quality", performance.quality));
      data_logger->record(state);

      // Terminate if simulation time has exceeded
      if (simulated_time > max_experiment_duration) terminate_mapping = true;
      /// TODO: Define termination condition for map quality
      if (terminate_mapping) {
        break;
      }
    }
    std::string elevation_output_path = result_directory + "/elevation_" + std::to_string(i) + ".png";
    writeGridmapToImage(adaptive_viewutility->getViewUtilityMap()->getGridMap(), "elevation", elevation_output_path);
    std::string prior_output_path = result_directory + "/geometric_prior_" + std::to_string(i) + ".png";
    writeGridmapToImage(adaptive_viewutility->getViewUtilityMap()->getGridMap(), "geometric_prior", prior_output_path);
    std::string saved_map_path = result_directory + "/gridmap_" + std::to_string(i) + ".bag";
    grid_map::GridMapRosConverter::saveToBag(adaptive_viewutility->getViewUtilityMap()->getGridMap(), saved_map_path,
                                             "/grid_map");
    std::cout << "[TestPlannerNode] Planner terminated experiment: " << i << std::endl;
    benchmark_results.emplace_back(data_logger);
  }

  // Evaluate benchmark results
  auto results_logger = std::make_shared<DataLogger>();
  results_logger->setPrintHeader(true);
  results_logger->setKeys({"timestamp", "coverage", "quality", "coverage_variance", "quality_variance", "coverage_min",
                           "coverage_max", "quality_min", "quality_max"});
  std::cout << "[TestPlannerNode] Number of benchmarks: " << benchmark_results.size() << std::endl;
  int num_data_points = benchmark_results[0]->count();
  for (int i = 0; i < benchmark_results[0]->count(); i++) {
    double time = std::any_cast<double &>(benchmark_results[0]->data()[i].at("timestamp"));
    double quality_mean{0.0};
    double coverage_mean{0.0};
    double quality_squared{0.0};
    double coverage_squared{0.0};
    double quality_min = std::numeric_limits<double>::infinity();
    double quality_max = -std::numeric_limits<double>::infinity();
    double coverage_min = std::numeric_limits<double>::infinity();
    double coverage_max = -std::numeric_limits<double>::infinity();
    int data_count{0};
    for (auto &result : benchmark_results) {
      if (i < result->data().size()) {
        double quality = std::any_cast<double &>(result->data()[i].at("quality"));
        quality_mean += quality;
        if (quality > quality_max) quality_max = quality;
        if (quality < quality_min) quality_min = quality;
        quality_squared += std::pow(quality, 2);
        double coverage = std::any_cast<double &>(result->data()[i].at("coverage"));
        coverage_mean += coverage;
        if (coverage > coverage_max) coverage_max = coverage;
        if (coverage < coverage_min) coverage_min = coverage;
        coverage_squared += std::pow(coverage, 2);
        data_count++;
      }
    }
    quality_mean = quality_mean / data_count;
    coverage_mean = coverage_mean / data_count;
    quality_squared = quality_squared / data_count;
    coverage_squared = coverage_squared / data_count;

    double quality_variance = quality_squared - std::pow(quality_mean, 2);
    double coverage_variance = coverage_squared - std::pow(coverage_mean, 2);
    std::unordered_map<std::string, std::any> average;
    average.insert(std::pair<std::string, double>("timestamp", time));
    average.insert(std::pair<std::string, double>("coverage", coverage_mean));
    average.insert(std::pair<std::string, double>("quality", quality_mean));
    average.insert(std::pair<std::string, double>("quality_variance", quality_variance));
    average.insert(std::pair<std::string, double>("coverage_variance", coverage_variance));
    average.insert(std::pair<std::string, double>("quality_variance", quality_variance));
    average.insert(std::pair<std::string, double>("coverage_min", coverage_min));
    average.insert(std::pair<std::string, double>("coverage_max", coverage_max));
    average.insert(std::pair<std::string, double>("quality_max", quality_max));
    average.insert(std::pair<std::string, double>("quality_min", quality_min));

    results_logger->record(average);
  }

  std::string benchmark_output_path = result_directory + "/active_benchmark.txt";
  results_logger->writeToFile(benchmark_output_path);
  std::cout << "[TestPlannerNode] Benchmark terminated" << std::endl;
  ros::spin();
  return 0;
}
