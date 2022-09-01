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
 * @brief Performance Tracker
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */
#include "adaptive_viewutility/performance_tracker.h"
#include <fstream>
#include <iostream>

PerformanceTracker::PerformanceTracker() {}

PerformanceTracker::~PerformanceTracker() {}

Metrics PerformanceTracker::Record(const double simulation_time, const grid_map::GridMap& gridmap) {
  int coverage_count{0};
  double map_quality{0.0};

  double resolution = gridmap.getResolution();
  double cell_area = std::pow(resolution, 2);
  int num_vaid_cells{0};
  for (grid_map::GridMapIterator iterator(gridmap); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index gridMapIndex = *iterator;
    const grid_map::Index index(*iterator);
    if (gridmap.at("roi", index) > 0.5) {
      if (gridmap.at("visibility", index) > 2) {
        coverage_count++;
      }
      map_quality += gridmap.at("geometric_prior", index);
      num_vaid_cells++;
    }
  }
  double roi_area = cell_area * double(num_vaid_cells);

  // Performance metrics
  Metrics metric;
  metric.time = simulation_time;
  metric.coverage = double(coverage_count) / double(num_vaid_cells);
  metric.quality = map_quality * cell_area / roi_area;
  performance_metrics_.push_back(metric);

  return metric;
}

void PerformanceTracker::Output(const std::string path) {
  // Write data to files

  std::ofstream output_file;
  output_file.open(path, std::ios::app);
  if (id_ == 0) {  // TODO: Make this nicer
    output_file << "id,timestamp,coverage,quality,padding,\n";
  }

  for (auto metric : performance_metrics_) {
    output_file << id_ << ",";
    output_file << metric.time << ",";
    output_file << metric.coverage << ",";
    output_file << metric.quality << ",";
    output_file << 0 << ",";
    output_file << "\n";
    id_++;
  }
  output_file.close();
  return;
}
