#include "terrain_planner/trajectory.h"

#include <gtest/gtest.h>
#include <iostream>

TEST(ManueverLibraryTest, getArcCenter) {
  Eigen::Vector2d segment_start(0.0, 0.0);
  Eigen::Vector2d segment_end(0.0, 2.0);
  Eigen::Vector2d expected_center(0.0, 1.0);

  Eigen::Vector2d arc_center = Trajectory::getArcCenter(segment_start, segment_end, 1.0);
  ASSERT_TRUE(arc_center.isApprox(expected_center));
  Eigen::Vector2d arc_center2 = Trajectory::getArcCenter(segment_start, segment_end, -1.0);
  ASSERT_TRUE(arc_center2.isApprox(expected_center));
}

TEST(ManueverLibraryTest, getArcProgress) {
  Eigen::Vector2d segment_start(0.0, 0.0);
  Eigen::Vector2d segment_end(0.0, 2.0);
  Eigen::Vector2d expected_center(0.0, 1.0);
  Eigen::Vector2d arc_center;
  double theta;
  theta = Trajectory::getArcProgress(arc_center, segment_start, segment_start, segment_end, 1.0);
  ASSERT_DOUBLE_EQ(theta, 0.0);
  ASSERT_TRUE(arc_center.isApprox(expected_center));
  theta = Trajectory::getArcProgress(arc_center, segment_end, segment_start, segment_end, 1.0);
  ASSERT_DOUBLE_EQ(theta, 1.0);
  theta = Trajectory::getArcProgress(arc_center, segment_start, segment_start, segment_end, -1.0);
  ASSERT_DOUBLE_EQ(theta, 0.0);
  theta = Trajectory::getArcProgress(arc_center, segment_end, segment_start, segment_end, -1.0);
  ASSERT_DOUBLE_EQ(theta, 1.0);

  Eigen::Vector2d test_position(0.0, -1.0);
  theta = Trajectory::getArcProgress(arc_center, test_position, segment_start, segment_end, 1.0);
  ASSERT_TRUE(arc_center.isApprox(expected_center));
  ASSERT_DOUBLE_EQ(theta, 0.0);
  theta = Trajectory::getArcProgress(arc_center, test_position, segment_start, segment_end, -1.0);
  ASSERT_DOUBLE_EQ(theta, 0.0);

  Eigen::Vector2d test_position2(1.0, 1.0);
  theta = Trajectory::getArcProgress(arc_center, test_position2, segment_start, segment_end, 1.0);
  ASSERT_DOUBLE_EQ(theta, 0.5);

  // Eigen::Vector2d test_position3(-1.0, 1.0);
  // theta = Trajectory::getArcProgress(arc_center, test_position3, segment_start, segment_end, -1.0);
  // ASSERT_DOUBLE_EQ(theta, 0.5);
}

TEST(ManueverLibraryTest, getLineProgress) {
  Eigen::Vector3d segment_start(0.0, 0.0, 0.0);
  Eigen::Vector3d segment_end(0.0, 0.0, 1.0);
  Eigen::Vector3d progress(0.0, 0.0, 0.5);

  double theta{0.0};
  theta = Trajectory::getLineProgress(progress, segment_start, segment_end);
  ASSERT_DOUBLE_EQ(theta, 0.5);

  theta = Trajectory::getLineProgress(segment_start, segment_start, segment_end);
  ASSERT_DOUBLE_EQ(theta, 0.0);

  theta = Trajectory::getLineProgress(segment_end, segment_start, segment_end);
  ASSERT_DOUBLE_EQ(theta, 1.0);
}
