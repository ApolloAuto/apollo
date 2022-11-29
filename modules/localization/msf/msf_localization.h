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

/**
 * @file msf_localization.h
 * @brief The class of MSFLocalization
 */

#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "gtest/gtest_prod.h"

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "modules/common_msgs/sensor_msgs/gnss_best_pose.pb.h"
#include "modules/common_msgs/sensor_msgs/gnss_raw_observation.pb.h"
#include "modules/common_msgs/sensor_msgs/imu.pb.h"
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"

#include "cyber/common/log.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/status/status.h"
#include "modules/localization/msf/local_integ/localization_integ.h"

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {

class LocalizationMsgPublisher;

/**
 * @class MSFLocalization
 *
 * @brief generate localization info based on MSF
 */
class MSFLocalization {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  MSFLocalization();

  apollo::common::Status Init();
  void InitParams();
  void OnPointCloud(const std::shared_ptr<drivers::PointCloud> &message);
  void OnRawImu(const std::shared_ptr<drivers::gnss::Imu> &imu_msg);
  void OnRawImuCache(const std::shared_ptr<drivers::gnss::Imu> &imu_msg);
  void OnGnssRtkObs(
      const std::shared_ptr<drivers::gnss::EpochObservation> &raw_obs_msg);
  void OnGnssRtkEph(
      const std::shared_ptr<drivers::gnss::GnssEphemeris> &gnss_orbit_msg);
  void OnGnssBestPose(
      const std::shared_ptr<drivers::gnss::GnssBestPose> &bestgnsspos_msg);
  void OnGnssHeading(
      const std::shared_ptr<drivers::gnss::Heading> &gnss_heading_msg);

  void SetPublisher(const std::shared_ptr<LocalizationMsgPublisher> &publisher);
  void OnLocalizationTimer();

 private:
  bool LoadGnssAntennaExtrinsic(const std::string &file_path, double *offset_x,
                                double *offset_y, double *offset_z,
                                double *uncertainty_x, double *uncertainty_y,
                                double *uncertainty_z);
  bool LoadImuVehicleExtrinsic(const std::string &file_path, double *quat_qx,
                               double *quat_qy, double *quat_qz,
                               double *quat_qw, Eigen::Vector3d *translation);
  bool LoadZoneIdFromFolder(const std::string &folder_path, int *zone_id);
  void CompensateImuVehicleExtrinsic(LocalizationEstimate *local_result);

 private:
  apollo::common::monitor::MonitorLogBuffer monitor_logger_;
  msf::LocalizationInteg localization_integ_;
  msf::LocalizationIntegParam localization_param_;
  msf::LocalizationMeasureState localization_state_;
  uint64_t pcd_msg_index_;

  // FRIEND_TEST(MSFLocalizationTest, InitParams);

  // rotation from the vehicle coord to imu coord
  Eigen::Quaternion<double> imu_vehicle_quat_;
  Eigen::Vector3d imu_vehicle_translation_;

  std::shared_ptr<LocalizationMsgPublisher> publisher_;
  std::shared_ptr<drivers::gnss::Imu> raw_imu_msg_;
  std::mutex mutex_imu_msg_;
  std::unique_ptr<cyber::Timer> localization_timer_ = nullptr;
};

}  // namespace localization
}  // namespace apollo
