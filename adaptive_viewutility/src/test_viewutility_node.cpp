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

#include "adaptive_viewutility/adaptive_viewutility.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "adaptive_viewutility");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  std::shared_ptr<AdaptiveViewUtility> adaptive_viewutility = std::make_shared<AdaptiveViewUtility>(nh, nh_private);

  std::string orientation_path;
  nh_private.param<std::string>("orientation_path", orientation_path, "images.txt");

  std::string file_path;
  nh_private.param<std::string>("file_path", file_path, "");

  std::string target_dir;
  nh_private.param<std::string>("dataset_dir", target_dir, "resources/");
  std::string text_path = target_dir + "/../images.txt";
  std::string camera_path = target_dir + "/../cameras.txt";

  bool replay;
  nh_private.param<bool>("replay", replay, true);

  // Add elevation map from GeoTIFF file defined in path
  adaptive_viewutility->LoadMap(file_path);

  bool initialized_time = false;
  ros::Time start_time_ = ros::Time::now();
  double start_time_seconds_ = 0.0;

  /// TODO: Iterate through images in directory

  for (int i = 428; i < 501; i++) {
    std::string image_file_name = "IMG_0" + std::to_string(i) + ".JPG";
    std::string pszSrcFilename = target_dir + "/" + image_file_name;
    std::cout << "Opening file: " << pszSrcFilename << std::endl;

    if (!adaptive_viewutility->AddViewPointFromImage(pszSrcFilename)) continue;  // No valid image

    Eigen::Vector4d camera_attitude;
    if (parseAttitudeFromText(text_path, image_file_name, camera_attitude)) {
      // Set attitude if it can be found from the file
      adaptive_viewutility->getViewPoints().back().setOrientation(camera_attitude);
      std::cout << "Added orientation to: " << image_file_name << std::endl;
    } else {
      std::cout << "No orientation available in: " << image_file_name << std::endl;
    }

    adaptive_viewutility->UpdateUtility(adaptive_viewutility->getViewPoints().back());
    double time_seconds = adaptive_viewutility->getViewPoints().back().getTime();

    if (replay) {
      if (!initialized_time) {
        start_time_ = ros::Time::now();
        start_time_seconds_ = time_seconds;
        initialized_time = true;
      } else {
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - start_time_).toSec();
        double delta_time = time_seconds - start_time_seconds_;
        ros::Duration(delta_time - dt).sleep();
      }
    }
    adaptive_viewutility->Visualize();
  }

  ros::spin();
  return 0;
}
