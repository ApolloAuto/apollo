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

/**
 * @file localization_integ_impl.h
 * @brief The class of LocalizationIntegImpl
 */

#pragma once

#include "modules/common/status/status.h"
#include "modules/localization/msf/local_integ/localization_gnss_process.h"
#include "modules/localization/msf/local_integ/localization_integ.h"
#include "modules/localization/msf/local_integ/localization_integ_process.h"
#include "modules/localization/msf/local_integ/localization_lidar_process.h"
#include "modules/localization/msf/local_integ/measure_republish_process.h"
#include "modules/localization/msf/local_integ/online_localization_expert.h"

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {
namespace msf {

class MeasureRepublishProcess;
class LocalizationIntegProcess;
class LocalizationGnssProcess;
class LocalizationLidarProcess;

/**
 * @class LocalizationIntegImpl
 *
 * @brief interface of msf localization
 */

class LocalizationIntegImpl {
 public:
  LocalizationIntegImpl();
  ~LocalizationIntegImpl();
  // Initialization.
  common::Status Init(const LocalizationIntegParam& params);

  // Lidar pcd process.
  void PcdProcess(const LidarFrame& lidar_frame);
  // Imu process.
  void RawImuProcessRfu(const ImuData& imu_data);

  // Gnss Info process.
  void RawObservationProcess(
      const drivers::gnss::EpochObservation& raw_obs_msg);
  void RawEphemerisProcess(const drivers::gnss::GnssEphemeris& gnss_orbit_msg);

  // gnss best pose process
  void GnssBestPoseProcess(const drivers::gnss::GnssBestPose& bestgnsspos_msg);

  // gnss heading process
  void GnssHeadingProcess(const drivers::gnss::Heading& gnssheading_msg);

  const LocalizationResult& GetLastestLidarLocalization() const;

  const LocalizationResult& GetLastestIntegLocalization() const;

  const LocalizationResult& GetLastestGnssLocalization() const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  void PcdProcessImpl(const LidarFrame& pcd_data);

  void ImuProcessImpl(const ImuData& imu_data);

  void RawObservationProcessImpl(
      const drivers::gnss::EpochObservation& raw_obs_msg);
  void RawEphemerisProcessImpl(
      const drivers::gnss::GnssEphemeris& gnss_orbit_msg);
  void GnssBestPoseProcessImpl(
      const drivers::gnss::GnssBestPose& bestgnsspos_msg);

  void GnssHeadingProcessImpl(const drivers::gnss::Heading& gnssheading_msg);

  void TransferGnssMeasureToLocalization(const MeasureData& measure,
                                         LocalizationEstimate* localization);

 private:
  MeasureRepublishProcess* republish_process_;
  LocalizationIntegProcess* integ_process_;
  LocalizationGnssProcess* gnss_process_;
  LocalizationLidarProcess* lidar_process_;

  LocalizationResult lastest_lidar_localization_;
  LocalizationResult lastest_integ_localization_;
  LocalizationResult lastest_gnss_localization_;

  bool is_use_gnss_bestpose_ = true;

  double imu_altitude_from_lidar_localization_ = 0.0;
  bool imu_altitude_from_lidar_localization_available_ = false;

  bool enable_lidar_localization_ = true;
  Eigen::Affine3d gnss_antenna_extrinsic_;
  OnlineLocalizationExpert expert_;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
