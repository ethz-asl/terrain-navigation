
#include <terrain_navigation_ros/geo_conversions.h>

Eigen::Vector3d GeoConversions::transformCoordinates(EPSG src_coord, EPSG tgt_coord,
                                                     const Eigen::Vector3d source_coordinates) {
  Eigen::Vector3d target_coordinates;
  if (src_coord == EPSG::WGS84 && tgt_coord == EPSG::CH1903_LV03) {
    GeoConversions::_forward(source_coordinates(0), source_coordinates(1), source_coordinates(2), target_coordinates(0),
                             target_coordinates(1), target_coordinates(2));
    return target_coordinates;
  } else if (src_coord == EPSG::CH1903_LV03 && tgt_coord == EPSG::WGS84) {
    GeoConversions::_reverse(source_coordinates(0), source_coordinates(1), source_coordinates(2), target_coordinates(0),
                             target_coordinates(1), target_coordinates(2));
    return target_coordinates;
  } else
    return GeoConversions::_transformUsingGDAL(src_coord, tgt_coord, source_coordinates);
};

Eigen::Vector3d _transformUsingGDAL(EPSG src_coord, EPSG tgt_coord, const Eigen::Vector3d source_coordinates) {
  OGRSpatialReference source, target;
  source.importFromEPSG(static_cast<int>(src_coord));
  target.importFromEPSG(static_cast<int>(tgt_coord));

  // Check if the EPSG are northing easting or easting northing.
  if (target.EPSGTreatsAsNorthingEasting()) {
    target.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
  }
  if (source.EPSGTreatsAsNorthingEasting()) {
    source.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
  }
  OGRPoint p;
  p.setX(source_coordinates(0));
  p.setY(source_coordinates(1));
  p.setZ(source_coordinates(2));
  p.assignSpatialReference(&source);

  p.transformTo(&target);
  Eigen::Vector3d target_coordinates(p.getX(), p.getY(), p.getZ());
  return target_coordinates;
};

static void GeoConversions::_forward(const double lat, const double lon, const double alt, double &y, double &x,
                                     double &h) {
  // 1. Convert the ellipsoidal latitudes φ and longitudes λ into arcseconds ["]
  const double lat_arc = lat * 3600.0;
  const double lon_arc = lon * 3600.0;

  // 2. Calculate the auxiliary values (differences of latitude and longitude relative to Bern in the unit [10000"]):
  //  φ' = (φ – 169028.66 ")/10000
  //  λ' = (λ – 26782.5 ")/10000
  const double lat_aux = (lat_arc - 169028.66) / 10000.0;
  const double lon_aux = (lon_arc - 26782.5) / 10000.0;

  // 3. Calculate projection coordinates in LV95 (E, N, h) or in LV03 (y, x, h)
  // E [m] = 2600072.37 + 211455.93 * λ' - 10938.51 * λ' * φ' - 0.36 * λ' * φ'2 - 44.54 * λ'3
  // y [m] = E – 2000000.00 N [m] = 1200147.07 + 308807.95 * φ' + 3745.25 * λ'2 + 76.63 * φ'2 - 194.56 * λ'2 * φ' +
  // 119.79 * φ'3 x [m] = N – 1000000.00
  // hCH [m] =hWGS – 49.55 + 2.73 * λ' + 6.94 * φ'
  const double E = 2600072.37 + 211455.93 * lon_aux - 10938.51 * lon_aux * lat_aux -
                   0.36 * lon_aux * std::pow(lat_aux, 2) - 44.54 * std::pow(lon_aux, 3);
  y = E - 2000000.00;
  const double N = 1200147.07 + 308807.95 * lat_aux + 3745.25 * std::pow(lon_aux, 2) + 76.63 * std::pow(lat_aux, 2) -
                   194.56 * std::pow(lon_aux, 2) * lat_aux + 119.79 * std::pow(lat_aux, 3);
  x = N - 1000000.00;

  h = alt - 49.55 + 2.73 * lon_aux + 6.94 * lat_aux;
};

static void GeoConversions::_reverse(const double y, const double x, const double h, double &lat, double &lon,
                                     double &alt) {
  // 1. Convert the projection coordinates E (easting) and N (northing) in LV95 (or y / x in LV03) into the civilian
  // system (Bern = 0 / 0) and express in the unit [1000 km]: E' = (E – 2600000 m)/1000000 = (y – 600000 m)/1000000
  // N' = (N – 1200000 m)/1000000 = (x – 200000 m)/1000000
  const double y_aux = (y - 600000.0) / 1000000.0;
  const double x_aux = (x - 200000.0) / 1000000.0;

  // 2. Calculate longitude λ and latitude φ in the unit [10000"]:
  //  λ' = 2.6779094 + 4.728982 * y' + 0.791484* y' * x' + 0.1306 * y' * x'2 - 0.0436 * y'3
  //  φ' = 16.9023892 + 3.238272 * x' - 0.270978 * y'2 - 0.002528 * x'2 - 0.0447 * y'2 * x' - 0.0140 * x'3
  // hWGS [m] = hCH + 49.55 - 12.60 * y' - 22.64 * x'
  const double lon_aux = 2.6779094 + 4.728982 * y_aux + 0.791484 * y_aux * x_aux + 0.1306 * y_aux * std::pow(x_aux, 2) -
                         0.0436 * std::pow(y_aux, 3);
  const double lat_aux = 16.9023892 + 3.238272 * x_aux - 0.270978 * std::pow(y_aux, 2) - 0.002528 * std::pow(x_aux, 2) -
                         0.0447 * std::pow(y_aux, 2) * x_aux - 0.0140 * std::pow(x_aux, 3);
  alt = h + 49.55 - 12.60 * y_aux - 22.64 * x_aux;

  lon = lon_aux * 100.0 / 36.0;
  lat = lat_aux * 100.0 / 36.0;
};
