/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/PointCloud2.h"
#include "yaml-cpp/yaml.h"

#include "modules/calibration/lidar_ex_checker/common/lidar_ex_checker_gflags.h"
#include "modules/calibration/lidar_ex_checker/lidar_ex_checker.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "ros/include/ros/ros.h"

namespace apollo {
namespace calibration {

using apollo::common::adapter::AdapterManager;
using apollo::common::Status;
using apollo::common::ErrorCode;

std::string LidarExChecker::Name() const {
  return "lidar_extrinsics_checker";
}

Status LidarExChecker::Init() {
  is_first_gps_msg_ = true;

  top_redundant_cloud_count_ = 0;
  bottom_redundant_cloud_count_ = 0;
  enough_data_ = false;

  cloud_count_ = 3;
  capture_distance_ = 10.0;

  position_type_ = 0;

  if (!LoadExtrinsics()) {
    return Status::OK();
  }

  std::cout << FLAGS_adapter_config_path << std::endl;
  AdapterManager::Init(FLAGS_adapter_config_path);

  CHECK(AdapterManager::GetGps()) << "gps is not initialized.";
  CHECK(AdapterManager::GetPointCloud()) << "PointCloud is not initialized.";
  AdapterManager::AddPointCloudCallback(&LidarExChecker::OnPointCloud, this);
  AdapterManager::AddGpsCallback(&LidarExChecker::OnGps, this);

  return Status::OK();
}

bool LidarExChecker::LoadExtrinsics() {
  std::string extrin_file = FLAGS_velodyne64_extrinsics_path;
  std::cout << extrin_file << std::endl;
  ifstream fi(extrin_file.c_str());
  if (!fi.good()) {
    std::cerr << "extrinsics file does not exist." << std::endl;
    return false;
  }

  YAML::Node node = YAML::LoadFile(extrin_file);

  Eigen::Quaterniond rotation(node["transform"]["rotation"]["w"].as<double>(),
                              node["transform"]["rotation"]["x"].as<double>(),
                              node["transform"]["rotation"]["y"].as<double>(),
                              node["transform"]["rotation"]["z"].as<double>());
  Eigen::Translation3d translation(
      node["transform"]["translation"]["x"].as<double>(),
      node["transform"]["translation"]["y"].as<double>(),
      node["transform"]["translation"]["z"].as<double>());
  extrinsics_ = translation * rotation;

  return true;
}

void LidarExChecker::VisualizeClouds() {
  boost::shared_ptr<pcl::visualization::PCLVisualizer> pcl_vis;
  pcl_vis.reset(new pcl::visualization::PCLVisualizer("3D Viewer"));

  // std::cout << "offset: " << std::endl << offset_.matrix() << std::endl;
  // std::cout << "extrinsics: " << std::endl << extrinsics_.matrix() <<
  // std::endl;
  // pcl::PointCloud<pcl::PointXYZ> save_cld;
  for (uint32_t i = 0; i < clouds_.size(); ++i) {
    pcl::PointCloud<PointXYZIT> cld = clouds_[i];
    pcl::PointCloud<pcl::PointXYZ>::Ptr tf_cld_ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    double timestamp = cld.points.back().timestamp;
    timestamp = round(timestamp * 100) / 100.0;
    // std::cout << timestamp << std::endl;
    Eigen::Affine3d pose = gps_poses_[timestamp];
    // std::cout << "pose" << i << ": " << std::endl << pose.matrix() <<
    // std::endl;
    for (uint32_t j = 0; j < cld.points.size(); ++j) {
      PointXYZIT pt = cld.points[j];
      Eigen::Vector3d pt_vec(pt.x, pt.y, pt.z);

      Eigen::Vector3d tf_pt_vec = pose * extrinsics_ * pt_vec;

      pcl::PointXYZ tf_pt;
      tf_pt.x = tf_pt_vec[0];
      tf_pt.y = tf_pt_vec[1];
      tf_pt.z = tf_pt_vec[2];
      tf_cld_ptr->points.push_back(tf_pt);

      // save_cld.points.push_back(tf_pt);
    }
    int r = rand() % 255;
    int g = rand() % 255;
    int b = rand() % 255;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler(
        tf_cld_ptr, r, g, b);
    pcl_vis->addPointCloud(tf_cld_ptr, handler, "clouds" + i);
  }
  // pcl::io::savePCDFileBinary("./result.pcd", save_cld);
  pcl_vis->spin();
}

void LidarExChecker::OnPointCloud(const sensor_msgs::PointCloud2& message) {
  if (top_redundant_cloud_count_ < 50) {
    top_redundant_cloud_count_++;
    return;
  }

  if (enough_data_) {
    bottom_redundant_cloud_count_++;
    if (bottom_redundant_cloud_count_ == 50) {
      VisualizeClouds();
    }
    return;
  }

  // if (position_type_ != 56) {
  //  return;
  //}

  Eigen::Vector3d position;
  Eigen::Affine3d pose = gps_poses_.rbegin()->second;
  position[0] = pose.translation().x();
  position[1] = pose.translation().y();
  position[2] = pose.translation().z();
  if ((position - last_position_).norm() < capture_distance_) {
    return;
  }

  pcl::PointCloud<PointXYZIT> cld;
  pcl::fromROSMsg(message, cld);

  pcl::PointCloud<PointXYZIT> tmp_cld;
  tmp_cld.header = cld.header;
  for (uint32_t i = 0; i < cld.points.size(); ++i) {
    if (pcl_isfinite(cld.points[i].x)) {
      tmp_cld.push_back(cld.points[i]);
    }
  }
  cld = tmp_cld;

  if (clouds_.size() < cloud_count_) {
    last_position_ = position;
    clouds_.push_back(cld);
    std::cout << "take " << clouds_.size() << " cloud(s)." << std::endl;
  }

  if (clouds_.size() >= cloud_count_) {
    enough_data_ = true;
  } else {
    enough_data_ = false;
  }
}

void LidarExChecker::OnGps(const ::apollo::localization::Gps& message) {
  if (message.has_localization()) {
    const auto pose_msg = message.localization();
    Eigen::Quaterniond rotation(
        pose_msg.orientation().qw(), pose_msg.orientation().qx(),
        pose_msg.orientation().qy(), pose_msg.orientation().qz());
    Eigen::Translation3d translation(pose_msg.position().x(),
                                     pose_msg.position().y(),
                                     pose_msg.position().z());
    Eigen::Affine3d pose = translation * rotation;

    if (is_first_gps_msg_) {
      is_first_gps_msg_ = false;
      last_position_[0] = translation.x();
      last_position_[1] = translation.y();
      last_position_[2] = translation.z();
      offset_ = pose.inverse();
    }
    Eigen::Affine3d new_pose = offset_ * pose;

    double timestamp = message.header().timestamp_sec();
    timestamp = round(timestamp * 100) / 100.0;

    gps_poses_.insert(std::make_pair(timestamp, new_pose));
  }
}

void LidarExChecker::OnInsStat() {
  position_type_ = 0;
}

Status LidarExChecker::Start() {
  return Status::OK();
}

void LidarExChecker::Stop() {}

}  // namespace calibration
}  // namespace apollo
