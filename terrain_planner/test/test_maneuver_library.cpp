#include "terrain_planner/trajectory.h"

#include <gtest/gtest.h>
#include <iostream>

TEST(ManueverLibraryTest, getArcCenter) {
  Eigen::Vector2d segment_start(0.0, 0.0);
  Eigen::Vector2d segment_end(0.0, 2.0);
  Eigen::Vector2d expected_center(0.0, 1.0);

  Eigen::Vector2d arc_center = TrajectorySegments::getArcCenter(segment_start, segment_end, 1.0);
  ASSERT_TRUE(arc_center.isApprox(expected_center));
  Eigen::Vector2d arc_center2 = TrajectorySegments::getArcCenter(segment_start, segment_end, -1.0);
  ASSERT_TRUE(arc_center2.isApprox(expected_center));
}

TEST(ManueverLibraryTest, getArcProgress) {
  Eigen::Vector2d segment_start(0.0, 0.0);
  Eigen::Vector2d segment_end(0.0, 2.0);
  Eigen::Vector2d expected_center(0.0, 1.0);
  Eigen::Vector2d arc_center;
  double theta;
  theta = TrajectorySegments::getArcProgress(arc_center, segment_start, segment_start, segment_end, 1.0);
  ASSERT_DOUBLE_EQ(theta, 0.0);
  ASSERT_TRUE(arc_center.isApprox(expected_center));
  theta = TrajectorySegments::getArcProgress(arc_center, segment_end, segment_start, segment_end, 1.0);
  ASSERT_DOUBLE_EQ(theta, 1.0);
  theta = TrajectorySegments::getArcProgress(arc_center, segment_start, segment_start, segment_end, -1.0);
  ASSERT_DOUBLE_EQ(theta, 0.0);
  theta = TrajectorySegments::getArcProgress(arc_center, segment_end, segment_start, segment_end, -1.0);
  ASSERT_DOUBLE_EQ(theta, 1.0);

  Eigen::Vector2d test_position(0.0, -1.0);
  theta = TrajectorySegments::getArcProgress(arc_center, test_position, segment_start, segment_end, 1.0);
  ASSERT_TRUE(arc_center.isApprox(expected_center));
  ASSERT_DOUBLE_EQ(theta, 0.0);
  theta = TrajectorySegments::getArcProgress(arc_center, test_position, segment_start, segment_end, -1.0);
  ASSERT_DOUBLE_EQ(theta, 0.0);

  Eigen::Vector2d test_position2(1.0, 1.0);
  theta = TrajectorySegments::getArcProgress(arc_center, test_position2, segment_start, segment_end, 1.0);
  ASSERT_DOUBLE_EQ(theta, 0.5);

  // Eigen::Vector2d test_position3(-1.0, 1.0);
  // theta = TrajectorySegments::getArcProgress(arc_center, test_position3, segment_start, segment_end, -1.0);
  // ASSERT_DOUBLE_EQ(theta, 0.5);
}
