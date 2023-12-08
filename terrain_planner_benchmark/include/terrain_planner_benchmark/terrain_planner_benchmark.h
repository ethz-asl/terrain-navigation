#ifndef TERRAIN_PLANNER_BENCHMARK_H
#define TERRAIN_PLANNER_BENCHMARK_H

#include <terrain_navigation/terrain_map.h>

#include "terrain_navigation/data_logger.h"
#include "terrain_planner/common.h"
#include "terrain_planner/terrain_ompl_rrt.h"
#include "terrain_planner/visualization.h"

struct BenchmarkResult {
  int id;
  bool found_solution;
  std::string planning_method;
  double path_length;
};

class TerrainPlannerBenchmark {
 public:
  TerrainPlannerBenchmark();
  ~TerrainPlannerBenchmark();
  void runBenchmark(const int num_experiments);
  void setMap(std::shared_ptr<TerrainMap> map) { map_ = std::move(map); };
  void writeResultstoFile(const std::string& file_path);

 private:
  std::shared_ptr<TerrainMap> map_;
  std::shared_ptr<DataLogger> data_logger;
  std::vector<BenchmarkResult> results;
};

#endif
