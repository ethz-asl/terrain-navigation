/****************************************************************************
 *
 *   Copyright (c) 2023 Jaeyoung Lim, Autonomous Systems Lab,
 *  ETH Zürich. All rights reserved.
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
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

#ifndef GEOCONVERSIONS_H
#define GEOCONVERSIONS_H

#include <math.h>
#include <Eigen/Dense>

enum class EPSG { ECEF = 4978, WGS84 = 4326, WGS84_32N = 32632, CH1903_LV03 = 21781, AT_GK_West = 31254 };

/**
 * @brief Helper function for transforming using gdal
 *
 * @param src_coord
 * @param tgt_coord
 * @param source_coordinates
 * @return Eigen::Vector3d
 */

class GeoConversions {
 public:
  GeoConversions();
  virtual ~GeoConversions();

  Eigen::Vector3d transformCoordinates(EPSG src_coord, EPSG tgt_coord, const Eigen::Vector3d source_coordinates);

 private:
  Eigen::Vector3d _transformUsingGDAL(EPSG src_coord, EPSG tgt_coord, const Eigen::Vector3d source_coordinates);

  /**
   * @brief Convert WGS84 (LLA) to LV03/CH1903
   *
   * @param lat latitude (degrees) WGS84
   * @param lon lontitude (degrees) WGS84
   * @param alt Altitude WGS84
   * @param x
   * @param y
   * @param h
   */
  static void _forward(const double lat, const double lon, const double alt, double &y, double &x, double &h);

  /**
   * @brief  LV03/CH1903 to Convert WGS84 (LLA)
   *
   * @param x
   * @param y
   * @param h
   * @param lat latitude
   * @param lon longitude
   * @param alt altitude
   */
  static void _reverse(const double y, const double x, const double h, double &lat, double &lon, double &alt);
};

#endif
