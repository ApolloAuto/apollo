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

#include "include/sins_struct.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/localization/msf/local_integ/localization_params.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/localization/proto/localization_status.pb.h"

/**
 * @namespace apollo::localization::msf
 * @brief apollo::localization::msf
 */
namespace apollo {
namespace localization {
namespace msf {

class OnlineLocalizationExpert {
 public:
  OnlineLocalizationExpert() = default;
  ~OnlineLocalizationExpert() = default;

  bool Init(const LocalizationIntegParam &param);

  void AddImu(const ImuData &data);
  void AddFusionLocalization(const LocalizationEstimate &data);
  void AddLidarLocalization(const LocalizationEstimate &data);
  void AddGnssBestPose(const drivers::gnss::GnssBestPose &msg,
                       const MeasureData &data);

  void GetFusionStatus(MsfStatus *msf_status, MsfSensorMsgStatus *sensor_status,
                       LocalizationIntegStatus *integstatus);
  void GetGnssStatus(MsfStatus *msf_status);

 private:
  void CheckImuDelayStatus(const double &cur_imu_time);
  void CheckImuMissingStatus(const double &cur_imu_time);
  void CheckGnssLidarMsfStatus(const double &cur_imu_time);
  void SetLocalizationStatus(const LocalizationEstimate &data);

 private:
  MsfStatus msf_status_;
  std::mutex msf_status_mutex_;

  MsfSensorMsgStatus sensor_status_;

  double latest_gnsspos_timestamp_ = 0.0;
  std::mutex latest_gnsspos_timestamp_mutex_;

  double latest_lidar_timestamp_ = 0.0;
  std::mutex latest_lidar_timestamp_mutex_;

  double imu_delay_time_threshold_1_ = 0.1;
  double imu_delay_time_threshold_2_ = 0.05;
  double imu_delay_time_threshold_3_ = 0.02;

  double imu_missing_time_threshold_1_ = 0.1;
  double imu_missing_time_threshold_2_ = 0.05;
  double imu_missing_time_threshold_3_ = 0.01;

  double bestgnsspose_loss_time_threshold_ = 2.0;
  double lidar_loss_time_threshold_ = 2.0;

  double localization_std_x_threshold_1_ = 0.15;
  double localization_std_y_threshold_1_ = 0.15;

  double localization_std_x_threshold_2_ = 0.3;
  double localization_std_y_threshold_2_ = 0.3;

  LocalizationIntegStatus integ_status_;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
