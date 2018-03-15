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

#include "modules/perception/tool/export_sensor_data/export_sensor_data.h"

#include <fstream>
#include <iostream>

#include "pcl/io/pcd_io.h"
#include "pcl_conversions/pcl_conversions.h"

#include "modules/common/log.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/common/pose_util.h"
#include "modules/perception/onboard/transform_input.h"

DEFINE_string(lidar_path, "modules/perception/tool/export_sensor_data/lidar/",
              "lidar path");
DEFINE_string(radar_path, "modules/perception/tool/export_sensor_data/radar/",
              "radar path");

namespace apollo {
namespace perception {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::adapter::AdapterManager;
using pcl_util::PointCloud;
using pcl_util::PointCloudPtr;

std::string ExportSensorData::Name() const { return "ExportSensorData"; }

Status ExportSensorData::Init() {
  AdapterManager::Init(FLAGS_perception_adapter_config_filename);
  CHECK(AdapterManager::GetPointCloud()) << "PointCloud is not initialized.";
  AdapterManager::AddPointCloudCallback(&ExportSensorData::OnPointCloud, this);
  CHECK(AdapterManager::GetContiRadar()) << "Radar is not initialized.";
  AdapterManager::AddContiRadarCallback(&ExportSensorData::OnRadar, this);
  CHECK(AdapterManager::GetLocalization())
      << "Localization is not initialized.";
  AdapterManager::AddLocalizationCallback(&ExportSensorData::OnLocalization,
                                          this);
  localization_buffer_.set_capacity(FLAGS_localization_buffer_size);

  /// load radar-camera extrinsic
  std::string radar_extrinsic_path = FLAGS_radar_extrinsic_file;
  Eigen::Affine3d radar_extrinsic;
  if (!LoadExtrinsic(radar_extrinsic_path, &radar_extrinsic)) {
    AERROR << "Failed to load extrinsic from file " << radar_extrinsic_path;
    return Status(ErrorCode::PERCEPTION_ERROR_TF, "Failed to load extrinsic");
  }
  AINFO << "get radar extrinsic succ. pose: \n" << radar_extrinsic.matrix();
  /// load camera-velodyne extrinsic
  std::string short_camera_extrinsic_path = FLAGS_short_camera_extrinsic_file;
  Eigen::Affine3d short_camera_extrinsic;
  if (!LoadExtrinsic(short_camera_extrinsic_path, &short_camera_extrinsic)) {
    AERROR << "Failed to load extrinsic from file "
           << short_camera_extrinsic_path;
    return Status(ErrorCode::PERCEPTION_ERROR_TF, "Failed to load extrinsic");
  }
  AINFO << "get short camera  extrinsic succ. pose: \n"
        << short_camera_extrinsic.matrix();
  /// get radar-velodyne extrinsic
  radar2velodyne_extrinsic_ =
      short_camera_extrinsic.matrix() * radar_extrinsic.matrix();

  return Status::OK();
}

void ExportSensorData::WriteRadar(const std::string& file_pre,
                                  const ContiRadar& radar_obs) {
  std::string filename = file_pre + ".radar";
  std::fstream fout(filename.c_str(), std::ios::out | std::ios::binary);
  if (!radar_obs.SerializeToOstream(&fout)) {
    AERROR << "Failed to write radar msg.";
    return;
  }
  fout.close();
}

void ExportSensorData::WritePose(const std::string& file_pre,
                                 const double timestamp, const int seq_num,
                                 const Eigen::Matrix4d& pose) {
  std::string filename = file_pre + ".pose";
  std::fstream fout(filename.c_str(), std::ios::out | std::ios::binary);
  if (!fout.is_open()) {
    AINFO << "Failed to write pose.";
  }
  Eigen::Matrix3f mat3f = pose.block<3, 3>(0, 0).cast<float>();
  Eigen::Quaternionf quaternion(mat3f);
  fout << std::setprecision(16) << seq_num << " " << timestamp << " "
       << pose(0, 3) << " " << pose(1, 3) << " " << pose(2, 3) << " "
       << quaternion.x() << " " << quaternion.y() << " " << quaternion.z()
       << " " << quaternion.w() << std::endl;
  fout.close();
}

void ExportSensorData::WriteVelocityInfo(const std::string& file_pre,
                                         const double& timestamp,
                                         const int seq_num,
                                         const Eigen::Vector3f& velocity) {
  std::string filename = file_pre + ".velocity";
  std::fstream fout(filename.c_str(), std::ios::out | std::ios::binary);
  if (!fout.is_open()) {
    AINFO << "Failed to write velocity.";
  }
  fout << std::setprecision(16) << seq_num << " " << timestamp << " "
       << velocity(0) << " " << velocity(1) << " " << velocity(2) << std::endl;
  fout.close();
}

void ExportSensorData::WritePCD(const std::string& file_pre,
                                const sensor_msgs::PointCloud2& in_msg) {
  pcl_util::PointCloudPtr cloud(new pcl_util::PointCloud);
  TransPointCloudToPCL(in_msg, &cloud);
  std::string filename = file_pre + ".pcd";
  try {
    pcl::io::savePCDFileASCII(filename, *cloud);
  } catch (const std::exception& e) {
    AERROR << "Something wrong, check the file path first.";
    return;
  }
}

void ExportSensorData::TransPointCloudToPCL(
    const sensor_msgs::PointCloud2& in_msg,
    pcl_util::PointCloudPtr* out_cloud) {
  // transform from ros to pcl
  pcl::PointCloud<pcl_util::PointXYZIT> in_cloud;
  pcl::fromROSMsg(in_msg, in_cloud);
  // transform from xyzit to xyzi
  pcl_util::PointCloudPtr& cloud = *out_cloud;
  cloud->header = in_cloud.header;
  cloud->width = in_cloud.width;
  cloud->height = in_cloud.height;
  cloud->is_dense = in_cloud.is_dense;
  cloud->sensor_origin_ = in_cloud.sensor_origin_;
  cloud->sensor_orientation_ = in_cloud.sensor_orientation_;
  cloud->points.resize(in_cloud.points.size());
  size_t points_num = 0;
  for (size_t idx = 0; idx < in_cloud.size(); ++idx) {
    pcl_util::PointXYZIT& pt = in_cloud.points[idx];
    if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z) &&
        !std::isnan(pt.intensity)) {
      cloud->points[points_num].x = pt.x;
      cloud->points[points_num].y = pt.y;
      cloud->points[points_num].z = pt.z;
      cloud->points[points_num].intensity = pt.intensity;
      points_num++;
    }
  }
  cloud->points.resize(points_num);
}

void ExportSensorData::OnPointCloud(const sensor_msgs::PointCloud2& message) {
  static int seq_num = 0;
  ++seq_num;
  AINFO << "process OnPointCloud.";
  const double kTimeStamp = message.header.stamp.toSec();

  /// get velodyne2world transfrom
  std::shared_ptr<Eigen::Matrix4d> velodyne_trans =
      std::make_shared<Eigen::Matrix4d>();
  if (!GetVelodyneTrans(kTimeStamp, velodyne_trans.get())) {
    AERROR << "failed to get velodyne trans at timestamp: "
           << GLOG_TIMESTAMP(kTimeStamp);
    return;
  }

  std::string str = boost::lexical_cast<std::string>(kTimeStamp);
  std::string file_pre = FLAGS_lidar_path + str;
  AINFO << "lidar file pre: " << file_pre;
  // save point cloud and pose
  WritePCD(file_pre, message);
  WritePose(file_pre, kTimeStamp, seq_num, *velodyne_trans);
}

void ExportSensorData::OnRadar(const ContiRadar& radar_obs) {
  AINFO << "process radar.";
  static int seq_num = 0;
  ++seq_num;
  ContiRadar radar_obs_proto = radar_obs;
  double timestamp = radar_obs_proto.header().timestamp_sec();
  double unix_timestamp = timestamp;
  const double cur_time = common::time::Clock::NowInSeconds();
  const double start_latency = (cur_time - unix_timestamp) * 1e3;
  AINFO << "FRAME_STATISTICS:Radar:Start:msg_time[" << GLOG_TIMESTAMP(timestamp)
        << "]:cur_time[" << GLOG_TIMESTAMP(cur_time) << "]:cur_latency["
        << start_latency << "]";
  // 0. correct radar timestamp
  timestamp -= 0.07;
  auto* header = radar_obs_proto.mutable_header();
  header->set_timestamp_sec(timestamp);
  header->set_radar_timestamp(timestamp * 1e9);

  conti_id_expansion_.UpdateTimestamp(timestamp);
  conti_id_expansion_.ExpandIds(&radar_obs_proto);

  if (fabs(timestamp - 0.0) < 10e-6) {
    AERROR << "Error timestamp: " << GLOG_TIMESTAMP(timestamp);
    return;
  }
  ADEBUG << "recv radar msg: [timestamp: " << GLOG_TIMESTAMP(timestamp)
         << " num_raw_obstacles: " << radar_obs_proto.contiobs_size() << "]";

  std::shared_ptr<Eigen::Matrix4d> velo2world_pose =
      std::make_shared<Eigen::Matrix4d>();
  if (!GetVelodyneTrans(timestamp, velo2world_pose.get())) {
    AERROR << "Failed to get radar trans at timestamp: "
           << GLOG_TIMESTAMP(timestamp);
    return;
  }
  std::shared_ptr<Eigen::Matrix4d> radar2world_pose =
      std::make_shared<Eigen::Matrix4d>();
  *radar2world_pose = *velo2world_pose * radar2velodyne_extrinsic_;
  AINFO << "get radar trans pose succ. pose: \n" << *radar2world_pose;

  Eigen::Vector3f car_linear_speed;
  // 3. get car car_linear_speed
  if (!GetCarLinearSpeed(timestamp, &car_linear_speed)) {
    AERROR << "Failed to call get_car_linear_speed. [timestamp: "
           << GLOG_TIMESTAMP(timestamp);
    return;
  }

  std::string str = boost::lexical_cast<std::string>(timestamp);
  std::string file_pre = FLAGS_radar_path + str;
  // save radar, pose, odometry
  AINFO << "radar file pre: " << file_pre;
  WriteRadar(file_pre, radar_obs_proto);
  WritePose(file_pre, timestamp, seq_num, *radar2world_pose);
  WriteVelocityInfo(file_pre, timestamp, seq_num, car_linear_speed);
}

void ExportSensorData::OnLocalization(
    const apollo::localization::LocalizationEstimate& localization) {
  double timestamp = localization.header().timestamp_sec();
  AINFO << "localization timestamp:" << GLOG_TIMESTAMP(timestamp);
  LocalizationPair localization_pair;
  localization_pair.first = timestamp;
  localization_pair.second = localization;
  localization_buffer_.push_back(localization_pair);
}

bool ExportSensorData::GetCarLinearSpeed(double timestamp,
                                         Eigen::Vector3f* car_linear_speed) {
  MutexLock lock(&mutex_);
  if (car_linear_speed == nullptr) {
    AERROR << "Param car_linear_speed NULL error.";
    return false;
  }
  if (localization_buffer_.empty()) {
    AWARN << "Rosmsg buffer is empty.";
    return false;
  }
  if (localization_buffer_.front().first - 0.1 > timestamp) {
    AWARN << "Timestamp (" << GLOG_TIMESTAMP(timestamp)
          << ") is earlier than the oldest "
          << "timestamp (" << localization_buffer_.front().first << ").";
    return false;
  }
  if (localization_buffer_.back().first + 0.1 < timestamp) {
    AWARN << "Timestamp (" << GLOG_TIMESTAMP(timestamp)
          << ") is newer than the latest "
          << "timestamp (" << localization_buffer_.back().first << ").";
    return false;
  }
  // loop to find nearest
  double distance = 1e9;
  int idx = localization_buffer_.size() - 1;
  for (; idx >= 0; --idx) {
    double temp_distance = fabs(timestamp - localization_buffer_[idx].first);
    if (temp_distance >= distance) {
      break;
    }
    distance = temp_distance;
  }
  const auto& velocity =
      localization_buffer_[idx].second.pose().linear_velocity();
  (*car_linear_speed)[0] = velocity.x();
  (*car_linear_speed)[1] = velocity.y();
  (*car_linear_speed)[2] = velocity.z();
  return true;
}

}  // namespace perception
}  // namespace apollo
