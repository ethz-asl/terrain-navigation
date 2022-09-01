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

#include "airsim_client/airsim_client.h"

AirsimClient::AirsimClient() { client_.confirmConnection(); }

AirsimClient::~AirsimClient(){};

Eigen::Matrix3d AirsimClient::quat2RotMatrix(const Eigen::Vector4d &q) {
  Eigen::Matrix3d rotmat;
  rotmat << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3), 2 * q(1) * q(2) - 2 * q(0) * q(3),
      2 * q(0) * q(2) + 2 * q(1) * q(3),

      2 * q(0) * q(3) + 2 * q(1) * q(2), q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
      2 * q(2) * q(3) - 2 * q(0) * q(1),

      2 * q(1) * q(3) - 2 * q(0) * q(2), 2 * q(0) * q(1) + 2 * q(2) * q(3),
      q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
  return rotmat;
}

Eigen::Vector4d AirsimClient::quatMultiplication(const Eigen::Vector4d &q, const Eigen::Vector4d &p) {
  Eigen::Vector4d quat;
  quat << p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3), p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2),
      p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1), p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);
  return quat;
}

void AirsimClient::getPose(Eigen::Vector3d &position, Eigen::Vector4d &attitude) {
  msr::airlib::Pose vehicle_pose = client_.simGetVehiclePose();
  Eigen::Vector3d pos_ned(vehicle_pose.position.x(), vehicle_pose.position.y(), vehicle_pose.position.z());
  Eigen::Matrix3d rotmat = AirsimClient::quat2RotMatrix(q_ned2enu_);
  position = rotmat * pos_ned;

  Eigen::Vector4d att_ned(vehicle_pose.orientation.w(), vehicle_pose.orientation.x(), vehicle_pose.orientation.y(),
                          vehicle_pose.orientation.z());
  Eigen::Vector4d q_camera = Eigen::Vector4d(std::cos(0.25 * M_PI), 0.0, std::sin(0.25 * M_PI), 0.0);
  Eigen::Vector4d att_camera = AirsimClient::quatMultiplication(att_ned, q_camera);
  attitude << att_camera(0), att_camera(1), -att_camera(2), -att_camera(3);
}

void AirsimClient::setPose(const Eigen::Vector3d &pos, const Eigen::Vector4d &att, std::string &file_name) {
  /// Airsim coordinate system is defined in NED. Therefore, we need to tranform our coordinates into NED from ENU
  Eigen::Vector4d att_ned(att(0), att(1), -att(2), -att(3));
  Eigen::Vector4d q_camera = Eigen::Vector4d(std::cos(-0.25 * M_PI), 0.0, std::sin(-0.25 * M_PI), 0.0);

  Eigen::Matrix3d rotmat = AirsimClient::quat2RotMatrix(q_ned2enu_);
  Eigen::Vector3d pos_ned = rotmat.inverse() * pos;
  Eigen::Vector4d att_camera = AirsimClient::quatMultiplication(att_ned, q_camera);

  msr::airlib::Vector3r p(pos_ned(0), pos_ned(1), pos_ned(2));
  msr::airlib::Quaternionr o(att_camera(0), att_camera(1), att_camera(2), att_camera(3));

  client_.simSetVehiclePose(msr::airlib::Pose(p, o), true);
  //
  std::vector<msr::airlib::ImageCaptureBase::ImageRequest> request = {
      msr::airlib::ImageCaptureBase::ImageRequest("", msr::airlib::ImageCaptureBase::ImageType::Scene, false, false)};
  const std::vector<msr::airlib::ImageCaptureBase::ImageResponse> &dummy_response = client_.simGetImages(request);

  const std::vector<msr::airlib::ImageCaptureBase::ImageResponse> &response = client_.simGetImages(request);

  if (response.size() > 0) {
    std::cout << "Enter path with ending separator to save images (leave empty for no save)" << std::endl;

    for (const msr::airlib::ImageCaptureBase::ImageResponse &image_info : response) {
      cv::Mat cv_image = cv::Mat(image_info.height, image_info.width, CV_8UC3);
      memcpy(cv_image.data, image_info.image_data_uint8.data(), image_info.image_data_uint8.size() * sizeof(char));

      if (image_directory_path_ != "") {
        file_name = std::to_string(image_info.time_stamp);
        std::string file_path = common_utils::FileSystem::combine(image_directory_path_, file_name) + ".jpeg";

        imwrite(file_path + ".jpeg", cv_image);  // A JPG FILE IS BEING SAVED

        char gps_tag_command[1024];
        char north_south = 'N', east_west = 'E';
        /// TODO: Convert coordinates using gdal
        Eigen::Vector3d home_position_lv03(kDefaultHomeX, kDefaultHomeY, kDefaultHomeAltitude);
        Eigen::Vector3d position_lv03 = pos + home_position_lv03;
        Eigen::Vector3d position_wgs84 =
            transformCoordinates(AIRSIM_ESPG::CH1903_LV03, AIRSIM_ESPG::WGS84, position_lv03);
        double lon = position_wgs84(0);
        double lat = position_wgs84(1);
        double altitude = position_wgs84(2);
        snprintf(gps_tag_command, sizeof(gps_tag_command),
                 "exiftool -gpslatituderef=%c -gpslongituderef=%c -gpsaltituderef=above -gpslatitude=%.9lf "
                 "-gpslongitude=%.9lf"
                 " -gpsdatetime=now -gpsmapdatum=WGS-84"
                 " -datetimeoriginal=now -gpsdop=0.8"
                 " -gpsmeasuremode=3-d -gpssatellites=13 -gpsaltitude=%.3lf -overwrite_original %s &>/dev/null",
                 north_south, east_west, lat, lon, altitude, (file_path + ".jpeg").c_str());
        if (system(gps_tag_command) < 0) {
          std::cout << "gps tag command failed" << std::endl;
          return;
        }
      }
    }
  }
}
