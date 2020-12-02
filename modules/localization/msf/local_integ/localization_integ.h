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
 * @file localization_integ.h
 * @brief The class of LocalizationInteg
 */

#pragma once

#include "localization_msf/gnss_struct.h"
#include "localization_msf/sins_struct.h"
#include "modules/common/status/status.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
#include "modules/drivers/gnss/proto/heading.pb.h"
#include "modules/drivers/gnss/proto/imu.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/localization/msf/local_integ/localization_lidar.h"
#include "modules/localization/msf/local_integ/localization_params.h"
#include "modules/localization/proto/localization.pb.h"

/**
 * @namespace apollo::localization::msf
 * @brief apollo::localization::msf
 */
namespace apollo {
namespace localization {
namespace msf {

class LocalizationIntegImpl;

/**
 * @class LocalizationInteg
 *
 * @brief interface of msf localization
 */

class LocalizationInteg {
 public:
  LocalizationInteg();
  ~LocalizationInteg();
  // Initialization.
  common::Status Init(const LocalizationIntegParam &params);

  // Lidar pcd process.
  void PcdProcess(const drivers::PointCloud &message);
  // Raw Imu process.
  // void CorrectedImuProcess(const Imu& imu_msg);
  void RawImuProcessFlu(const drivers::gnss::Imu &imu_msg);
  void RawImuProcessRfu(const drivers::gnss::Imu &imu_msg);
  // Gnss Info process.
  void RawObservationProcess(
      const drivers::gnss::EpochObservation &raw_obs_msg);
  void RawEphemerisProcess(const drivers::gnss::GnssEphemeris &gnss_orbit_msg);
  // gnss best pose process
  void GnssBestPoseProcess(const drivers::gnss::GnssBestPose &bestgnsspos_msg);
  // gnss heading process
  void GnssHeadingProcess(const drivers::gnss::Heading &gnss_heading_msg);

  const LocalizationResult &GetLastestLidarLocalization() const;
  const LocalizationResult &GetLastestIntegLocalization() const;
  const LocalizationResult &GetLastestGnssLocalization() const;

 protected:
  void TransferImuFlu(const drivers::gnss::Imu &imu_msg, ImuData *imu_data);

  void TransferImuRfu(const drivers::gnss::Imu &imu_msg, ImuData *imu_rfu);

 private:
  LocalizationIntegImpl *localization_integ_impl_;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
