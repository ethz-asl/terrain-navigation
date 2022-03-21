//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef TERRAIN_OMPL_RRT_H
#define TERRAIN_OMPL_RRT_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <tf/transform_broadcaster.h>

#include <stdio.h>
#include <cstdlib>
#include <sstream>
#include <string>

#include <octomap/octomap.h>
#include <Eigen/Dense>

#include <terrain_planner/ompl_setup.h>
#include "terrain_navigation/terrain_map.h"

using namespace std;
using namespace Eigen;

class TerrainOmplRrt {
 public:
  TerrainOmplRrt();
  virtual ~TerrainOmplRrt();

  void setupProblem();
  void setBounds(Eigen::Vector3d& lower_bound, Eigen::Vector3d& upper_bound) {
    lower_bound_ = lower_bound;
    upper_bound_ = upper_bound;
  }
  void setMap(std::shared_ptr<TerrainMap> map) { map_ = map; }
  bool Solve(const Eigen::Vector3d& start, const Eigen::Vector3d& goal, std::vector<Eigen::Vector3d>& path);
  void solutionPathToTrajectoryPoints(ompl::geometric::PathGeometric& path,
                                      std::vector<Eigen::Vector3d>& trajectory_points) const;
  std::shared_ptr<ompl::base::PlannerData> getPlannerData() { return planner_data_; };
  ompl::OmplSetup& getProblemSetup() { return problem_setup_; };

 private:
  ompl::OmplSetup problem_setup_;
  std::shared_ptr<TerrainMap> map_;
  std::shared_ptr<ompl::base::PlannerData> planner_data_;
  Eigen::Vector3d lower_bound_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d upper_bound_{Eigen::Vector3d::Zero()};
};

#endif
