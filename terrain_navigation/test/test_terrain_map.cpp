#include <gtest/gtest.h>

#include <iostream>

#include "terrain_navigation/terrain_map.h"

TEST(TerrainMapTest, geoTransform) {
  // Depending on Gdal versions, lon lat order are reversed
#if GDAL_VERSION_MAJOR > 2
  Eigen::Vector3d berghaus_wgs84(46.7209147, 9.9219592, 488.0);
#else
  Eigen::Vector3d berghaus_wgs84(9.9219592, 46.7209147, 488.0);
#endif
  Eigen::Vector3d berghaus_lv03(789823.96735451114, 177416.47911055354, 440.3752994351089);
  Eigen::Vector3d tranformed_lv03 = TerrainMap::transformCoordinates(ESPG::WGS84, ESPG::CH1903_LV03, berghaus_wgs84);
  EXPECT_NEAR(tranformed_lv03(0), berghaus_lv03(0), 0.0001);
  EXPECT_NEAR(tranformed_lv03(1), berghaus_lv03(1), 0.0001);
  EXPECT_NEAR(tranformed_lv03(2), berghaus_lv03(2), 0.0001);
  Eigen::Vector3d tranformed_wgs84 = TerrainMap::transformCoordinates(ESPG::CH1903_LV03, ESPG::WGS84, berghaus_lv03);
  EXPECT_NEAR(tranformed_wgs84(0), berghaus_wgs84(0), 0.0001);
  EXPECT_NEAR(tranformed_wgs84(1), berghaus_wgs84(1), 0.0001);
  EXPECT_NEAR(tranformed_wgs84(2), berghaus_wgs84(2), 0.0001);
  std::string wkt =
      "PROJCS[\"CH1903 / LV03\", GEOGCS[\"CH1903\", DATUM[\"CH1903\", SPHEROID[\"Bessel 1841\", 6377397.155, "
      "299.1528128, AUTHORITY[\"EPSG\", \"7004\"]], TOWGS84[674.374, 15.056, 405.346, 0, 0, 0, 0], AUTHORITY[\"EPSG\", "
      "\"6149\"]], PRIMEM[\"Greenwich\", 0, AUTHORITY[\"EPSG\", \"8901\"]], UNIT[\"degree\", 0.0174532925199433, "
      "AUTHORITY[\"EPSG\", \"9122\"]], AUTHORITY[\"EPSG\", "
      "\"4149\"]],PROJECTION[\"Hotine_Oblique_Mercator_Azimuth_Center\"], PARAMETER[\"latitude_of_center\", "
      "46.95240555555556], PARAMETER[\"longitude_of_center\", 7.439583333333333], PARAMETER[\"azimuth\", 90], "
      "PARAMETER[\"rectified_grid_angle\", 90], PARAMETER[\"scale_factor\", 1], PARAMETER[\"false_easting\", 600000], "
      "PARAMETER[\"false_northing\", 200000], UNIT[\"metre\", 1, AUTHORITY[\"EPSG\", \"9001\"]], AXIS[\"Y\", EAST], "
      "AXIS[\"X\", NORTH], AUTHORITY[\"EPSG\", \"21781\"]]";
  Eigen::Vector3d tranformed_lv032 = TerrainMap::transformCoordinates(ESPG::WGS84, wkt, berghaus_wgs84);
  EXPECT_NEAR(tranformed_lv032(0), berghaus_lv03(0), 0.0001);
  EXPECT_NEAR(tranformed_lv032(1), berghaus_lv03(1), 0.0001);
  EXPECT_NEAR(tranformed_lv032(2), berghaus_lv03(2), 0.0001);
}
