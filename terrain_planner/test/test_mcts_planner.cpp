#include "terrain_planner/mcts_planner.h"

#include <gtest/gtest.h>
#include <iostream>

TEST(MCTSPlannerTest, DecendTree) {
  auto mcts_planner = std::make_shared<MctsPlanner>();
  auto terrain_map = std::make_shared<TerrainMap>();
  mcts_planner->setTerrainMap(terrain_map);

  EXPECT_TRUE(true);
}
