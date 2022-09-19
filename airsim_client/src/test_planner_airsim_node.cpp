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
#include "airsim_client/airsim_client.h"
#include "terrain_navigation/profiler.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "adaptive_viewutility");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  std::vector<std::shared_ptr<DataLogger>> benchmark_results;

  std::string file_path, output_dir_path;
  int num_experiments;
  double snapshot_interval;  // Interval (seconds) that the package will keep track of
  double max_experiment_duration;
  bool viewpoint_noise{false};
  nh_private.param<std::string>("file_path", file_path, "");
  nh_private.param<int>("num_experiments", num_experiments, 1);
  nh_private.param<double>("snapshot_interval", snapshot_interval, 20.0);
  nh_private.param<double>("max_experiment_duration", max_experiment_duration, 200.0);
  nh_private.param<bool>("viewpoint_noise", viewpoint_noise, false);
  nh_private.param<std::string>("output_directory_path", output_dir_path, "output");

  ros::Publisher camera_path_pub = nh.advertise<nav_msgs::Path>("camera_path", 1, true);
  ros::Publisher camera_pose_pub = nh.advertise<geometry_msgs::PoseArray>("camera_poses", 1, true);
  ros::Publisher viewpoint_pub = nh.advertise<visualization_msgs::MarkerArray>("viewpoints", 1, true);

  std::shared_ptr<AirsimClient> airsim_client = std::make_shared<AirsimClient>();

  std::string image_directory = output_dir_path + "/images";
  airsim_client->setImageDirectory(image_directory);

  /// set Current state of vehicle
  /// TODO: Randomly generate initial position
  Eigen::Vector3d airsim_player_start = airsim_client->getPlayerStart();
  std::vector<double> time_vector;

  for (int i = 0; i < num_experiments; i++) {
    std::shared_ptr<AdaptiveViewUtility> adaptive_viewutility = std::make_shared<AdaptiveViewUtility>(nh, nh_private);

    // Add elevation map from GeoTIFF file defined in path
    adaptive_viewutility->LoadMap(file_path);
    adaptive_viewutility->getViewUtilityMap()->TransformMap(airsim_player_start);

    grid_map::Polygon polygon;

    // Hah! we have access to the map reference. be careful!
    grid_map::GridMap &map = adaptive_viewutility->getViewUtilityMap()->getGridMap();
    const Eigen::Vector2d map_pos = map.getPosition();
    const double map_width_x = map.getLength().x();
    const double map_width_y = map.getLength().y();

    polygon.setFrameId(map.getFrameId());
    double roi_portion = 0.5;
    polygon.addVertex(
        grid_map::Position(map_pos(0) - roi_portion * map_width_x, map_pos(1) - roi_portion * map_width_y));
    polygon.addVertex(
        grid_map::Position(map_pos(0) + roi_portion * map_width_x, map_pos(1) - roi_portion * map_width_y));
    polygon.addVertex(
        grid_map::Position(map_pos(0) + roi_portion * map_width_x, map_pos(1) + roi_portion * map_width_y));
    polygon.addVertex(
        grid_map::Position(map_pos(0) - roi_portion * map_width_x, map_pos(1) + roi_portion * map_width_y));
    grid_map::Polygon offset_polygon = polygon;
    offset_polygon.offsetInward(1.0);

    Eigen::Vector2d start_pos_2d = offset_polygon.getVertex(0);
    adaptive_viewutility->getViewUtilityMap()->SetRegionOfInterest(polygon);

    Eigen::Vector3d vehicle_pos(start_pos_2d(0), start_pos_2d(1), 100.0);
    double elevation = adaptive_viewutility->getViewUtilityMap()->getGridMap().atPosition(
        "elevation", Eigen::Vector2d(vehicle_pos(0), vehicle_pos(1)));
    vehicle_pos(2) = vehicle_pos(2) + elevation;
    Eigen::Vector3d vehicle_vel(15.0, 0.0, 0.0);
    // adaptive_viewutility->InitializeVehicleFromMap(vehicle_pos, vehicle_vel);
    std::cout << "Initial Position: " << vehicle_pos.transpose() << std::endl;
    std::cout << "Initial Velocity: " << vehicle_vel.transpose() << std::endl;
    adaptive_viewutility->setCurrentState(vehicle_pos, vehicle_vel);
    Profiler pipeline_perf("Planner Loop");
    std::shared_ptr<PerformanceTracker> performance_tracker = std::make_shared<PerformanceTracker>();

    std::shared_ptr<DataLogger> camera_logger = std::make_shared<DataLogger>();
    camera_logger->setKeys({"file", "X", "Y", "Z"});
    camera_logger->setSeparator(" ");

    std::shared_ptr<DataLogger> data_logger = std::make_shared<DataLogger>();
    data_logger->setKeys({"timestamp", "coverage", "quality", "image_count", "position_x", "position_y", "position_z",
                          "attitude_w", "attitude_x", "attitude_y", "attitude_z"});

    std::shared_ptr<DataLogger> camera_logger = std::make_shared<DataLogger>();
    camera_logger->setKeys({"file", "X", "Y", "Z"});
    camera_logger->setSeparator(" ");

    std::shared_ptr<DataLogger> data_logger = std::make_shared<DataLogger>();
    data_logger->setKeys({"timestamp", "coverage", "quality", "image_count", "position_x", "position_y", "position_z",
                          "attitude_w", "attitude_x", "attitude_y", "attitude_z"});

    bool terminate_mapping = false;
    double simulated_time{0.0};
    double elapsed_time{0.0};
    int snapshot_index{0};
    std::default_random_engine random_generator_;
    std::normal_distribution<double> standard_normal_distribution_(0.0, 45.0 / 180.0 * M_PI);
    int image_count{0};
    while (true) {
      pipeline_perf.tic();
      adaptive_viewutility->generateMotionPrimitives();
      adaptive_viewutility->estimateViewUtility();

      Trajectory reference_trajectory = adaptive_viewutility->getBestPrimitive();

      Trajectory first_segment;
      for (int i = 0; i < 5; i++) {
        // TODO: Add noise
        if (viewpoint_noise) {
          double theta = standard_normal_distribution_(random_generator_);
          std::cout << "Theta: " << theta * 180.0 / M_PI << std::endl;
          Eigen::Vector3d axis =
              Eigen::Vector3d(getRandom(-1.0, 1.0), getRandom(-1.0, 1.0), getRandom(-1.0, 1.0)).normalized();
          axis = axis * std::sin(theta / 2.0);
          Eigen::Vector4d disturbance_att;
          disturbance_att << std::cos(theta / 2.0), axis(0), axis(1), axis(2);
          std::cout << "  - original attitude: " << reference_trajectory.states[i].attitude.transpose() << std::endl;
          reference_trajectory.states[i].attitude =
              AirsimClient::quatMultiplication(reference_trajectory.states[i].attitude, disturbance_att);
          std::cout << "  - modified attitude: " << reference_trajectory.states[i].attitude.transpose() << std::endl;
        }
        first_segment.states.push_back(reference_trajectory.states[i]);

        std::string image_path;
        airsim_client->setPose(reference_trajectory.states[i].position, reference_trajectory.states[i].attitude,
                               image_path);
        image_count++;

        Metrics performance =
            performance_tracker->Record(simulated_time, adaptive_viewutility->getViewUtilityMap()->getGridMap());

        std::cout << "image path: " << image_path << std::endl;
        std::unordered_map<std::string, std::any> state;
        state.insert(std::pair<std::string, double>("timestamp", performance.time));
        state.insert(std::pair<std::string, double>("coverage", performance.coverage));
        state.insert(std::pair<std::string, double>("quality", performance.quality));
        state.insert(std::pair<std::string, double>("image_count", image_count));
        state.insert(std::pair<std::string, double>("position_x", reference_trajectory.states[i].position.x()));
        state.insert(std::pair<std::string, double>("position_y", reference_trajectory.states[i].position.y()));
        state.insert(std::pair<std::string, double>("position_z", reference_trajectory.states[i].position.z()));
        state.insert(std::pair<std::string, double>("attitude_w", reference_trajectory.states[i].attitude(0)));
        state.insert(std::pair<std::string, double>("attitude_x", reference_trajectory.states[i].attitude(1)));
        state.insert(std::pair<std::string, double>("attitude_y", reference_trajectory.states[i].attitude(2)));
        state.insert(std::pair<std::string, double>("attitude_z", reference_trajectory.states[i].attitude(3)));
        data_logger->record(state);

        std::unordered_map<std::string, std::any> camera_state;
        camera_state.insert(std::pair<std::string, std::string>("file", image_path));
        camera_state.insert(std::pair<std::string, double>("X", reference_trajectory.states[i].position.x()));
        camera_state.insert(std::pair<std::string, double>("Y", reference_trajectory.states[i].position.y()));
        camera_state.insert(std::pair<std::string, double>("Z", reference_trajectory.states[i].position.z()));
        camera_logger->record(camera_state);

        simulated_time += 2.0;
        // Terminate if simulation time has exceeded
        if (simulated_time > max_experiment_duration) {
          terminate_mapping = true;
          break;
        }

        ros::Duration(0.1).sleep();
      }

      adaptive_viewutility->UpdateUtility(first_segment);

      adaptive_viewutility->setCurrentState(first_segment.states.back().position, first_segment.states.back().velocity);

      std::cout << "Planning time: " << pipeline_perf.toc() << std::endl;
      time_vector.push_back(pipeline_perf.toc());
      adaptive_viewutility->MapPublishOnce();
      adaptive_viewutility->ViewpointPublishOnce(camera_path_pub, camera_pose_pub);
      adaptive_viewutility->publishViewpoint(viewpoint_pub);

      double planning_horizon = adaptive_viewutility->getViewPlanner()->getPlanningHorizon();
      elapsed_time += planning_horizon;
      if (elapsed_time >= snapshot_interval) {
        elapsed_time = 0.0;
        std::string saved_map_path =
            image_directory + "/gridmap_" + std::to_string(static_cast<int>(snapshot_index)) + ".bag";
        grid_map::GridMapRosConverter::saveToBag(adaptive_viewutility->getViewUtilityMap()->getGridMap(),
                                                 saved_map_path, "/grid_map");
        snapshot_index++;
      }
      if (terminate_mapping) {
        break;
      }
    }
    std::cout << "[TestPlannerNode] write to file" << std::endl;
    camera_logger->writeToFile(output_dir_path + "/camera.txt");
    std::cout << "[TestPlannerNode] Planner terminated experiment: " << i << std::endl;
    benchmark_results.emplace_back(data_logger);
    // Evaluate benchmark results
    auto results_logger = std::make_shared<DataLogger>();
    results_logger->setPrintHeader(true);
    results_logger->setKeys({"timestamp", "coverage", "quality", "coverage_variance", "quality_variance",
                             "coverage_min", "coverage_max", "quality_min", "quality_max", "image_count"});
    std::cout << "[TestPlannerNode] Number of benchmarks: " << benchmark_results.size() << std::endl;
    int num_data_points = benchmark_results[0]->count();
    for (int i = 0; i < benchmark_results[0]->count(); i++) {
      double time = std::any_cast<double &>(benchmark_results[0]->data()[i].at("timestamp"));
      int num_images = std::any_cast<double &>(benchmark_results[0]->data()[i].at("image_count"));
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
      average.insert(std::pair<std::string, double>("image_count", num_images));

      results_logger->record(average);
    }
    std::string benchmark_output_path = output_dir_path + "/coverage_benchmark.txt";
    results_logger->writeToFile(benchmark_output_path);
  }
  for (int i = 0; i < benchmark_results.size(); i++) {
    benchmark_results[i]->writeToFile(output_dir_path + "/benchamrk_raw_" + std::to_string(i) + ".txt");
  }
  std::cout << "[TestPlannerNode] Planner terminated" << std::endl;
  // ros::spin();
  return 0;
}
