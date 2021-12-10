#ifndef COMMON_H
#define COMMON_H

#include <Eigen/Dense>
#include <fstream>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

geometry_msgs::Pose vector3d2PoseMsg(const Eigen::Vector3d position, const Eigen::Vector4d orientation) {
  geometry_msgs::Pose encode_msg;

  encode_msg.orientation.w = orientation(0);
  encode_msg.orientation.x = orientation(1);
  encode_msg.orientation.y = orientation(2);
  encode_msg.orientation.z = orientation(3);
  encode_msg.position.x = position(0);
  encode_msg.position.y = position(1);
  encode_msg.position.z = position(2);
  return encode_msg;
}

geometry_msgs::PoseStamped vector3d2PoseStampedMsg(const Eigen::Vector3d position, const Eigen::Vector4d orientation) {
  geometry_msgs::PoseStamped encode_msg;

  encode_msg.header.stamp = ros::Time::now();
  encode_msg.header.frame_id = "map";
  encode_msg.pose.orientation.w = orientation(0);
  encode_msg.pose.orientation.x = orientation(1);
  encode_msg.pose.orientation.y = orientation(2);
  encode_msg.pose.orientation.z = orientation(3);
  encode_msg.pose.position.x = position(0);
  encode_msg.pose.position.y = position(1);
  encode_msg.pose.position.z = position(2);
  return encode_msg;
}

visualization_msgs::Marker utility2MarkerMsg(const double utility, const Eigen::Vector3d position, int id) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = position(0);
  marker.pose.position.y = position(1);
  marker.pose.position.z = position(2);
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  double vis_utility;
  if (utility < 1.0) {
    vis_utility = 1.0 / 1;
  } else {
    vis_utility = utility / 1;
  }
  marker.scale.x = vis_utility;
  marker.scale.y = vis_utility;
  marker.scale.z = vis_utility;
  marker.color.a = 0.5;  // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  return marker;
}

visualization_msgs::Marker normals2ArrowsMsg(const Eigen::Vector3d &position, const Eigen::Vector3d &normal, int id) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "normals";
  marker.id = id;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = position(0);
  marker.pose.position.y = position(1);
  marker.pose.position.z = position(2);

  const Eigen::Vector3d arrow_direction = Eigen::Vector3d::UnitX();
  const Eigen::Vector3d normal_vector = normal.normalized();
  Eigen::Vector3d u = arrow_direction.cross(normal_vector);
  double theta = std::acos(normal_vector.dot(arrow_direction));

  // TODO: Convert arrow vector to orientation of arrow
  marker.pose.orientation.x = sin(0.5 * theta) * u(0);
  marker.pose.orientation.y = sin(0.5 * theta) * u(1);
  marker.pose.orientation.z = sin(0.5 * theta) * u(2);
  marker.pose.orientation.w = cos(0.5 * theta);
  marker.scale.x = 5.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  return marker;
}

double GetTimeInSeconds(std::string date_time) {
  std::stringstream ss(date_time);
  std::string tagged_time;
  std::string tagged_date;

  ss >> tagged_date >> tagged_time;

  std::stringstream ss_time(tagged_time);
  std::vector<std::string> time_hour;

  while (ss_time.good()) {
    std::string substr;
    std::getline(ss_time, substr, ':');
    time_hour.push_back(substr);
  }

  return 3600.0 * std::stof(time_hour[0]) + 60.0 * std::stof(time_hour[1]) + std::stoi(time_hour[2]);
}

double StringToGeoReference(std::string &exif_tag) {
  std::stringstream ss(exif_tag);
  std::vector<std::string> result;
  while (ss.good()) {
    std::string substr;
    std::getline(ss, substr, '(');
    std::getline(ss, substr, ')');
    result.push_back(substr);
  }
  double output;
  if (result.size() >= 4) {
    /// TODO: Check precision and coordinate frame of this conversion
    output = std::stod(result[0]) + 0.0166667 * std::stod(result[1]) + 0.000277778 * std::stod(result[2]);
  } else {
    output = std::stod(result[0]);
  }
  return output;
}

bool parseAttitudeFromText(std::string text_path, std::string image_file, Eigen::Vector4d &attitude) {
  bool parse_result;

  std::ifstream file(text_path);
  std::string str;

  // Look for the image file name in the path
  while (getline(file, str)) {
    if (str.find(image_file) != std::string::npos) {
      std::stringstream ss(str);
      std::vector<std::string> camera_pose;
      camera_pose.resize(9);
      // #   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
      ss >> camera_pose[0] >> camera_pose[1] >> camera_pose[2] >> camera_pose[3] >> camera_pose[4];
      attitude << std::stof(camera_pose[1]), std::stof(camera_pose[2]), std::stof(camera_pose[3]),
          std::stof(camera_pose[4]);
      return true;
    }
  }
  return false;
}

double getRandom(double min, double max) {
  return std::abs(max - min) * static_cast<double>(rand()) / static_cast<double>(RAND_MAX) + std::min(max, min);
}

#endif
