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
 * @brief Node to test planner in the view utiltiy map
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include <pcl/features/normal_3d.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>  // for KdTree
#include <pcl/surface/gp3.h>
#include "adaptive_viewutility/adaptive_viewutility.h"

#define M2CM 100

int main(int argc, char **argv) {
  ros::init(argc, argv, "adaptive_viewutility");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  std::string file_path, output_file_path;
  nh_private.param<std::string>("file_path", file_path, "");
  nh_private.param<std::string>("output_file_path", output_file_path, "");

  std::cout << "Starting mesh trimmer: " << file_path << std::endl;

  // pcl::PolygonMesh mesh;
  pcl::PCLPointCloud2 mesh;
  pcl::io::loadOBJFile(file_path, mesh);

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromPCLPointCloud2(mesh, *input_cloud);

  pcl::CropBox<pcl::PointXYZ> cropBoxFilter(true);
  cropBoxFilter.setInputCloud(input_cloud);

  const Eigen::Vector3f player_origin(-37447.859375, -72312.984375, 28677.371094);
  const Eigen::Vector3f mesh_origin(-260096.0, -260096.0, 129462.992188);
  // Wrong location!
  // Vertex coordinates 36201.0859, 11844.017, 15441.48
  // Vertex coodrinates 35967.769, 11845.818, 15747.11035

  // Vertex coordinates -29662.3, 16794.2, 58886.3
  // Vertex coordinates -21888, 15611.4, -41984
  // Vertex new!

  const Eigen::Vector3f local_position(0.0, -100.0, 0.0);
  Eigen::Vector3f offset = Eigen::Vector3f(-29311.08791, 16763.689453, -58477.63679);

  const float width = 200 * M2CM;
  const float altitude = 1000 * M2CM;

  Eigen::Vector4f min_pt(offset(0) - 0.5 * width, offset(1) - 0.5 * altitude, offset(2) - 0.5 * width, 0.0f);
  Eigen::Vector4f max_pt(offset(0) + 0.5 * width, offset(1) + 0.5 * altitude, offset(2) + 0.5 * width, 1.0f);

  // Cropbox slighlty bigger then bounding box of points
  cropBoxFilter.setMin(min_pt);
  cropBoxFilter.setMax(max_pt);

  // Cloud
  cropBoxFilter.filter(*cropped_cloud);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cropped_cloud);
  n.setInputCloud(cropped_cloud);
  n.setSearchMethod(tree);
  n.setKSearch(20);
  n.compute(*normals);

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(*cropped_cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud(cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius(1000.0);

  // Set typical values for the parameters
  gp3.setMu(10.0);
  gp3.setMaximumNearestNeighbors(50);
  gp3.setMaximumSurfaceAngle(2 * M_PI);  // 45 degrees
  gp3.setMinimumAngle(0.0);              // 10 degrees
  gp3.setMaximumAngle(2 * M_PI);         // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud(cloud_with_normals);
  gp3.setSearchMethod(tree2);
  gp3.reconstruct(triangles);

  pcl::io::saveOBJFile(output_file_path, triangles, 10);
  return 0;
}
