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
 * @brief Adaptive view utility estimation node
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#ifndef AIRSIM_CLIENT_H
#define AIRSIM_CLIENT_H

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif  // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include <gdal/cpl_string.h>
#include <gdal/gdal.h>
#include <gdal/gdal_priv.h>
#include <gdal/ogr_p.h>
#include <gdal/ogr_spatialref.h>

#include "common/common_utils/FileSystem.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

// Zurich Irchel Park
static constexpr const double kDefaultHomeX = 683565.21;     // LV03/CH1903
static constexpr const double kDefaultHomeY = 250246.85;     // rad
static constexpr const double kDefaultHomeAltitude = 488.0;  // meters
enum class ESPG { ECEF = 4978, WGS84 = 4326, WGS84_32N = 32632, CH1903_LV03 = 21781 };

class AirsimClient {
 public:
  AirsimClient();
  virtual ~AirsimClient();
  static Eigen::Matrix3d quat2RotMatrix(const Eigen::Vector4d &q);
  static Eigen::Vector4d quatMultiplication(const Eigen::Vector4d &q, const Eigen::Vector4d &p);
  void setPose(const Eigen::Vector3d &pos, const Eigen::Vector4d &att);
  void setImageDirectory(const std::string &image_directory) { image_directory_path_ = image_directory; };
  void getPose(Eigen::Vector3d &position, Eigen::Vector4d &attitude);
  Eigen::Vector3d getPlayerStart() { return player_start_; };
  static Eigen::Vector3d transformCoordinates(ESPG src_coord, ESPG tgt_coord,
                                              const Eigen::Vector3d source_coordinates) {
    OGRSpatialReference source, target;
    source.importFromEPSG(static_cast<int>(src_coord));
    target.importFromEPSG(static_cast<int>(tgt_coord));

    OGRPoint p;
    p.setX(source_coordinates(0));
    p.setY(source_coordinates(1));
    p.setZ(source_coordinates(2));
    p.assignSpatialReference(&source);

    p.transformTo(&target);
    Eigen::Vector3d target_coordinates(p.getX(), p.getY(), p.getZ());
    return target_coordinates;
  }

 private:
  msr::airlib::RpcLibClientBase client_;
  std::string image_directory_path_;
  double lat_home_{kDefaultHomeX};
  double lon_home_{kDefaultHomeY};
  double alt_home_{kDefaultHomeAltitude};
  // Original player_start(-374.47859375,  -723.12984375, 286.77371094)
  Eigen::Vector3d player_start_{Eigen::Vector3d(374.47859375, -723.12984375, -286.77371094)};
  Eigen::Vector4d q_ned2enu_{Eigen::Vector4d(std::cos(-0.5 * M_PI), std::sin(-0.5 * M_PI), 0.0, 0.0)};
};

#endif
