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

#include "modules/calibration/lidar_ex_checker/lidar_ex_checker.h"

#include "eigen_conversions/eigen_msg.h"
#include "pcl/io/pcd_io.h"
#include "pcl/visualization/cloud_viewer.h"
#include "pcl_conversions/pcl_conversions.h"
#include "tf2_ros/transform_listener.h"

#include "modules/calibration/lidar_ex_checker/common/lidar_ex_checker_gflags.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"

namespace apollo {
namespace calibration {

using apollo::common::adapter::AdapterManager;
using apollo::common::Status;
using apollo::common::ErrorCode;

std::string LidarExChecker::Name() const { return "lidar_extrinsics_checker"; }

Status LidarExChecker::Init() {
  is_first_gps_msg_ = true;

  top_redundant_cloud_count_ = 0;
  bottom_redundant_cloud_count_ = 0;
  enough_data_ = false;

  cloud_count_ = FLAGS_capture_cloud_count;
  capture_distance_ = FLAGS_capture_distance;

  position_type_ = 0;

  AdapterManager::Init(FLAGS_adapter_config_filename);

  CHECK(AdapterManager::GetGps()) << "GPS is not initialized.";
  CHECK(AdapterManager::GetPointCloud()) << "PointCloud is not initialized.";
  CHECK(AdapterManager::GetInsStat()) << "InsStat is not initialized.";
  AdapterManager::AddPointCloudCallback(&LidarExChecker::OnPointCloud, this);
  AdapterManager::AddGpsCallback(&LidarExChecker::OnGps, this);
  AdapterManager::AddInsStatCallback(&LidarExChecker::OnInsStat, this);

  return Status::OK();
}

bool LidarExChecker::GetExtrinsics() {
  static tf2_ros::Buffer tf2_buffer;
  static tf2_ros::TransformListener tf2Listener(tf2_buffer);

  std::string err_msg;

  if (!tf2_buffer.canTransform("novatel", "velodyne64", ros::Time(0),
                               ros::Duration(100), &err_msg)) {
    std::cerr << "Fail to get velodyne64 extrinsics for tf" << std::endl;
    return false;
  }

  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped =
      tf2_buffer.lookupTransform("novatel", "velodyne64", ros::Time(0));
  tf::transformMsgToEigen(transform_stamped.transform, extrinsics_);

  return true;
}

void LidarExChecker::VisualizeClouds() {
  if (!GetExtrinsics()) {
    return;
  }

  boost::shared_ptr<pcl::visualization::PCLVisualizer> pcl_vis;
  pcl_vis.reset(new pcl::visualization::PCLVisualizer("3D Viewer"));
  for (uint32_t i = 0; i < clouds_.size(); ++i) {
    pcl::PointCloud<PointXYZIT> cld = clouds_[i];
    pcl::PointCloud<pcl::PointXYZ>::Ptr tf_cld_ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    double timestamp = cld.points.back().timestamp;
    timestamp = round(timestamp * 100) / 100.0;
    Eigen::Affine3d pose = gps_poses_[timestamp];

    for (uint32_t j = 0; j < cld.points.size(); ++j) {
      PointXYZIT pt = cld.points[j];
      Eigen::Vector3d pt_vec(pt.x, pt.y, pt.z);
      Eigen::Vector3d tf_pt_vec = pose * extrinsics_ * pt_vec;

      pcl::PointXYZ tf_pt;
      tf_pt.x = tf_pt_vec[0];
      tf_pt.y = tf_pt_vec[1];
      tf_pt.z = tf_pt_vec[2];
      tf_cld_ptr->points.push_back(tf_pt);
    }
    uint32_t seed = static_cast<uint32_t>(timestamp);
    int r = rand_r(&seed) % 255;
    int g = rand_r(&seed) % 255;
    int b = rand_r(&seed) % 255;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler(
        tf_cld_ptr, r, g, b);
    pcl_vis->addPointCloud(tf_cld_ptr, handler, "clouds" + i);
  }
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

  if (position_type_ != 56) {
    return;
  }

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
  }

  if (clouds_.size() >= cloud_count_) {
    enough_data_ = true;
  } else {
    enough_data_ = false;
  }
}

void LidarExChecker::OnGps(const localization::Gps& message) {
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

void LidarExChecker::OnInsStat(const drivers::gnss::InsStat& message) {
  position_type_ = message.pos_type();
}

Status LidarExChecker::Start() { return Status::OK(); }

void LidarExChecker::Stop() {}

}  // namespace calibration
}  // namespace apollo
