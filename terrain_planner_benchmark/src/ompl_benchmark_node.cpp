/****************************************************************************
 *
 *   Copyright (c) 2022
 *   Jaeyoung Lim, Autonomous Systems Lab, ETH Zurich All rights reserved.
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
 * @brief Node to test planner in the view utiltiy map
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include <ompl-1.5/ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/tools/benchmark/Benchmark.h>

#include <ros/ros.h>

#include "terrain_planner/terrain_ompl_rrt.h"

ompl::base::PlannerPtr ConfigureRRTPlanner(const ompl::base::SpaceInformationPtr &si) {
  ompl::geometric::RRTstar *rrt = new ompl::geometric::RRTstar(si);
  rrt->setName("RRT*");
  // rrt->setRange(100.0);
  return ompl::base::PlannerPtr(rrt);
}

ompl::base::PlannerPtr ConfigureBITPlanner(const ompl::base::SpaceInformationPtr &si) {
  ompl::geometric::BITstar *bit = new ompl::geometric::BITstar(si);
  bit->setName("BIT*");
  bit->setPruning(false);  // Pruning needs to be set to false, otherwise results in a segfault
  return ompl::base::PlannerPtr(bit);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ompl_rrt_planner");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  std::string map_path, color_file_path, output_file_path;
  nh_private.param<std::string>("map_path", map_path, "");
  nh_private.param<std::string>("color_file_path", color_file_path, "");
  nh_private.param<std::string>("output_file_path", output_file_path, "");

  auto terrain_map = std::make_shared<TerrainMap>();
  terrain_map->initializeFromGeotiff(map_path, false);
  if (!color_file_path.empty()) {  // Load color layer if the color path is nonempty
    terrain_map->addColorFromGeotiff(color_file_path);
  }
  terrain_map->AddLayerDistanceTransform(50.0, "distance_surface");
  terrain_map->AddLayerOffset(150.0, "max_elevation");

  std::vector<Eigen::Vector3d> path;
  double terrain_altitude{100.0};

  auto planner = std::make_shared<TerrainOmplRrt>();

  planner->setMap(terrain_map);
  /// TODO: Get bounds from gridmap
  planner->setBoundsFromMap(terrain_map->getGridMap());

  const Eigen::Vector2d map_pos = terrain_map->getGridMap().getPosition();
  const double map_width_x = terrain_map->getGridMap().getLength().x();
  const double map_width_y = terrain_map->getGridMap().getLength().y();

  Eigen::Vector3d start{Eigen::Vector3d(map_pos(0) + 0.3 * map_width_x, map_pos(1) - 0.3 * map_width_y, 0.0)};
  start(2) = terrain_map->getGridMap().atPosition("elevation", Eigen::Vector2d(start(0), start(1))) + terrain_altitude;
  Eigen::Vector3d goal{Eigen::Vector3d(map_pos(0) + 0.3 * map_width_x, map_pos(1) + 0.3 * map_width_y, 0.0)};
  goal(2) = terrain_map->getGridMap().atPosition("elevation", Eigen::Vector2d(goal(0), goal(1))) + terrain_altitude;
  planner->setupProblem(start, goal);

  // Create a state space for the space we are planning in
  // ompl::geometric::SimpleSetup ss(space);
  auto ss = *planner->getProblemSetup();

  // First we create a benchmark class:
  ompl::tools::Benchmark b(ss, "Planner Benchmarks");

  // Optionally, specify some benchmark parameters (doesn't change how the benchmark is run)
  // b.addExperimentParameter("num_dofs", "INTEGER", "6");
  // b.addExperimentParameter("num_obstacles", "INTEGER", "10");

  // For planners that we want to configure in specific ways,
  // the ompl::base::PlannerAllocator should be used:
  b.addPlannerAllocator(std::bind(&ConfigureRRTPlanner, std::placeholders::_1));
  b.addPlannerAllocator(std::bind(&ConfigureBITPlanner, std::placeholders::_1));
  // etc.

  // Now we can benchmark: 5 second time limit for each plan computation,
  // 100 MB maximum memory usage per plan computation, 50 runs for each planner
  // and true means that a text-mode progress bar should be displayed while
  // computation is running.
  ompl::tools::Benchmark::Request req;
  req.maxTime = 150.0;
  req.maxMem = 100.0;
  req.runCount = 3;
  req.displayProgress = true;
  b.benchmark(req);

  // This will generate a file of the form ompl_host_time.log
  b.saveResultsToFile();

  // Configure the problem to solve: set start state(s)
  // and goal representation
  // Everything must be set up to the point ss.solve()
  // can be called. Setting up a planner is not needed.
  return 0;
}
