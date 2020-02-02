/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#pragma once

#include <list>
#include <memory>
#include <string>

#include "Eigen/Geometry"

#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/localization/ndt/localization_pose_buffer.h"
#include "modules/localization/ndt/ndt_locator/lidar_locator_ndt.h"
#include "modules/localization/proto/gps.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/transform/buffer.h"

namespace apollo {
namespace localization {
namespace ndt {

struct LidarHeight {
  LidarHeight() : height(0.0), height_var(0.0) {}
  double height;
  double height_var;
};

struct TimeStampPose {
  double timestamp = 0.0;
  Eigen::Affine3d pose;
};

class NDTLocalization {
 public:
  NDTLocalization() {}
  ~NDTLocalization() { tf_buffer_ = nullptr; }
  /**@brief init configuration */
  void Init();
  /**@brief receive odometry message */
  void OdometryCallback(const std::shared_ptr<localization::Gps>& odometry_msg);
  /**@brief receive lidar pointcloud message */
  void LidarCallback(const std::shared_ptr<drivers::PointCloud>& lidar_msg);
  /**@brief receive ins status message */
  void OdometryStatusCallback(
      const std::shared_ptr<drivers::gnss::InsStat>& status_msg);
  /**@brief service start status */
  bool IsServiceStarted();
  /**@brief output localization result */
  void GetLocalization(LocalizationEstimate* localization) const;
  /**@brief get ndt localization result */
  void GetLidarLocalization(LocalizationEstimate* lidar_localization) const;
  /**@brief get localization status */
  void GetLocalizationStatus(LocalizationStatus* localization_status) const;
  /**@brief get zone id */
  inline int GetZoneId() const { return zone_id_; }
  /**@brief get online resolution for ndt localization*/
  inline double GetOnlineResolution() const { return online_resolution_; }

 private:
  /**@brief transfer pointcloud message to LidarFrame */
  void LidarMsgTransfer(const std::shared_ptr<drivers::PointCloud>& message,
                        LidarFrame* lidar_frame);
  /**@brief Load lidar extrinsics from file */
  bool LoadLidarExtrinsic(const std::string& file_path,
                          Eigen::Affine3d* lidar_extrinsic);
  /**@brief load lidar height from file */
  bool LoadLidarHeight(const std::string& file_path, LidarHeight* height);
  /**@brief load zone id from map folder */
  bool LoadZoneIdFromFolder(const std::string& folder_path, int* zone_id);
  /**@brief query forecast pose from tf2 according to timestamp*/
  bool QueryPoseFromTF(double time, Eigen::Affine3d* pose);
  /**@brief query forecast pose from odometry buffer, in case fail to get pose
   * from tf*/
  bool QueryPoseFromBuffer(double time, Eigen::Affine3d* pose);
  /**@brief check if odometry message is zero*/
  bool ZeroOdometry(const Eigen::Affine3d& pose);
  /**@brief fill header message for LocalizationEstimate struct */
  void FillLocalizationMsgHeader(LocalizationEstimate* localization);
  /**@brief fill pose message for LocalizationEstimate struct */
  void ComposeLocalizationEstimate(
      const Eigen::Affine3d& pose,
      const std::shared_ptr<localization::Gps>& odometry_msg,
      LocalizationEstimate* localization);
  void ComposeLidarResult(double time_stamp, const Eigen::Affine3d& pose,
                          LocalizationEstimate* localization);
  /**@brief fill localization status */
  void ComposeLocalizationStatus(const drivers::gnss::InsStat& status,
                                 LocalizationStatus* localization_status);
  /**@brief find nearest odometry status */
  bool FindNearestOdometryStatus(const double odometry_timestamp,
                                 drivers::gnss::InsStat* status);

 private:
  std::string module_name_ = "ndt_localization";
  LocalizationPoseBuffer pose_buffer_;

  transform::Buffer* tf_buffer_ = nullptr;
  std::string tf_target_frame_id_ = "";
  std::string tf_source_frame_id_ = "";

  LidarLocatorNdt lidar_locator_;
  int zone_id_ = 10;
  double max_height_ = 100.0;
  int localization_seq_num_ = 0;
  unsigned int resolution_id_ = 0;
  double online_resolution_ = 2.0;
  std::string map_path_ = "";
  LidarHeight lidar_height_;
  Eigen::Affine3d lidar_pose_;
  Eigen::Affine3d velodyne_extrinsic_;
  LocalizationEstimate lidar_localization_result_;
  double ndt_score_ = 0;
  unsigned int bad_score_count_ = 0;
  unsigned int bad_score_count_threshold_ = 10;
  double warnning_ndt_score_ = 1.0;
  double error_ndt_score_ = 2.0;
  bool is_service_started_ = false;

  std::list<TimeStampPose> odometry_buffer_;
  std::mutex odometry_buffer_mutex_;
  unsigned int odometry_buffer_size_ = 0;
  const unsigned int max_odometry_buffer_size_ = 100;

  LocalizationEstimate localization_result_;
  LocalizationStatus localization_status_;

  std::list<drivers::gnss::InsStat> odometry_status_list_;
  size_t odometry_status_list_max_size_ = 10;
  std::mutex odometry_status_list_mutex_;
  double odometry_status_time_diff_threshold_ = 1.0;

  bool ndt_debug_log_flag_ = false;
};

}  // namespace ndt
}  // namespace localization
}  // namespace apollo
