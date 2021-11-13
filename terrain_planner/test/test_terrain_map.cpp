#include "terrain_planner/terrain_map.h"

#include <gtest/gtest.h>
#include <iostream>

TEST(TerrainMapTest, geoTransform) {
  // Depending on Gdal versions, lon lat order are reversed
#if GDAL_VERSION_MAJOR > 2
  Eigen::Vector2d berghaus_wgs84(46.7209147, 9.9219592);
#else
  Eigen::Vector2d berghaus_wgs84(9.9219592, 46.7209147);
#endif
  Eigen::Vector2d berghaus_lv03(789823.97475067549, 177416.4901717098);
  Eigen::Vector2d tranformed_lv03 = TerrainMap::transformCoordinates(ESPG::WGS84, ESPG::CH1903_LV03, berghaus_wgs84);
  EXPECT_NEAR(tranformed_lv03(0), berghaus_lv03(0), 0.0001);
  EXPECT_NEAR(tranformed_lv03(1), berghaus_lv03(1), 0.0001);
  Eigen::Vector2d tranformed_wgs84 = TerrainMap::transformCoordinates(ESPG::CH1903_LV03, ESPG::WGS84, berghaus_lv03);
  EXPECT_NEAR(tranformed_wgs84(0), berghaus_wgs84(0), 0.0001);
  EXPECT_NEAR(tranformed_wgs84(1), berghaus_wgs84(1), 0.0001);
}
