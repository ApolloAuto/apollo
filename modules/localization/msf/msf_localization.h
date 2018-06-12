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

#ifndef MODULES_LOCALIZATION_MSF_MSF_LOCALIZATION_H_
#define MODULES_LOCALIZATION_MSF_MSF_LOCALIZATION_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest_prod.h"

#include "ros/include/ros/ros.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "sensor_msgs/PointCloud2.h"

#include "modules/drivers/gnss/proto/imu.pb.h"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/localization/proto/localization.pb.h"

#include "modules/common/log.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/status/status.h"
#include "modules/localization/localization_base.h"
#include "modules/localization/msf/local_integ/localization_integ.h"

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {

/**
 * @class MSFLocalization
 *
 * @brief generate localization info based on MSF
 */
class MSFLocalization : public LocalizationBase {
 public:
  MSFLocalization();

  /**
   * @brief module start function
   * @return start status
   */
  apollo::common::Status Start() override;

  /**
   * @brief module stop function
   * @return stop status
   */
  apollo::common::Status Stop() override;

 private:
  apollo::common::Status Init();
  void InitParams();
  void OnPointCloud(const sensor_msgs::PointCloud2 &message);
  void OnRawImu(const drivers::gnss::Imu &imu_msg);
  void OnGnssRtkObs(const drivers::gnss::EpochObservation &raw_obs_msg);
  void OnGnssRtkEph(const drivers::gnss::GnssEphemeris &gnss_orbit_msg);
  void OnGnssBestPose(const drivers::gnss::GnssBestPose &bestgnsspos_msg);

 private:
  bool LoadGnssAntennaExtrinsic(const std::string &file_path, double *offset_x,
                                double *offset_y, double *offset_z,
                                double *uncertainty_x, double *uncertainty_y,
                                double *uncertainty_z);
  bool LoadImuVehicleExtrinsic(const std::string &file_path,
                                double *quat_qx, double *quat_qy,
                                double *quat_qz, double *quat_qw);
  bool LoadZoneIdFromFolder(const std::string &folder_path, int *zone_id);

 private:
  apollo::common::monitor::MonitorLogger monitor_logger_;
  msf::LocalizationInteg localization_integ_;
  msf::LocalizationIntegParam localizaiton_param_;
  msf::LocalizationMeasureState localization_state_;
  uint64_t pcd_msg_index_;

  MeasureState latest_lidar_localization_status_;
  MeasureState latest_gnss_localization_status_;

  // FRIEND_TEST(MSFLocalizationTest, InitParams);

  // rotation from the vehicle coord to imu coord
  Eigen::Quaternion<double> imu_vehicle_quat_;
};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_MSF_MSF_LOCALIZATION_H_
