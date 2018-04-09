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

#include "modules/localization/msf/local_integ/localization_integ.h"
#include <map>
#include <list>
#include "modules/localization/msf/local_integ/localization_integ_impl.h"
#include "modules/localization/msf/local_integ/lidar_msg_transfer.h"
#include "modules/localization/msf/common/util/frame_transform.h"
#include "modules/localization/msf/common/util/time_conversion.h"
#include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace localization {
namespace msf {

using common::Status;

LocalizationInteg::LocalizationInteg() {
  localization_integ_impl_ = new LocalizationIntegImpl;
}

LocalizationInteg::~LocalizationInteg() {
  delete localization_integ_impl_;
}

Status LocalizationInteg::Init(
    const LocalizationIntegParam& params) {
  return localization_integ_impl_->Init(params);
}

void LocalizationInteg::PcdProcess(const sensor_msgs::PointCloud2& message) {
  LidarFrame lidar_frame;
  LidarMsgTransfer::Transfer(message, &lidar_frame);
  localization_integ_impl_->PcdProcess(lidar_frame);
  return;
}

void LocalizationInteg::RawImuProcessFlu(const drivers::gnss::Imu& imu_msg) {
  ImuData imu;
  TransferImuFlu(imu_msg, &imu);
  localization_integ_impl_->RawImuProcessRfu(imu);
  return;
}

void LocalizationInteg::RawImuProcessRfu(const drivers::gnss::Imu& imu_msg) {
  ImuData imu;
  TransferImuRfu(imu_msg, &imu);
  localization_integ_impl_->RawImuProcessRfu(imu);
  return;
}

void LocalizationInteg::RawObservationProcess(
    const drivers::gnss::EpochObservation& raw_obs_msg) {
  localization_integ_impl_->RawObservationProcess(raw_obs_msg);
  return;
}

void LocalizationInteg::RawEphemerisProcess(
    const drivers::gnss::GnssEphemeris& gnss_orbit_msg) {
  localization_integ_impl_->RawEphemerisProcess(gnss_orbit_msg);
  return;
}

void LocalizationInteg::GnssBestPoseProcess(
    const drivers::gnss::GnssBestPose& bestgnsspos_msg) {
  localization_integ_impl_->GnssBestPoseProcess(bestgnsspos_msg);
  return;
}

void LocalizationInteg::GetLastestLidarLocalization(
    LocalizationMeasureState *state,
    LocalizationEstimate *lidar_localization) {
  localization_integ_impl_->GetLastestLidarLocalization(
      state, lidar_localization);
  return;
}

void LocalizationInteg::GetLastestIntegLocalization(
    LocalizationMeasureState *state,
    LocalizationEstimate *integ_localization) {
  localization_integ_impl_->GetLastestIntegLocalization(
      state, integ_localization);
  return;
}

void LocalizationInteg::GetLastestGnssLocalization(
    LocalizationMeasureState *state,
    LocalizationEstimate *gnss_localization) {
  localization_integ_impl_->GetLastestGnssLocalization(
      state, gnss_localization);
  return;
}

void LocalizationInteg::GetLidarLocalizationList(
    std::list<LocalizationResult> *results) {
  localization_integ_impl_->GetLidarLocalizationList(results);
  return;
}

void LocalizationInteg::GetIntegLocalizationList(
    std::list<LocalizationResult> *results) {
  localization_integ_impl_->GetIntegLocalizationList(results);
  return;
}

void LocalizationInteg::GetGnssLocalizationList(
    std::list<LocalizationResult> *results) {
  localization_integ_impl_->GetGnssLocalizationList(results);
  return;
}

void LocalizationInteg::TransferImuRfu(const drivers::gnss::Imu &imu_msg,
                                       ImuData *imu_rfu) {
  double measurement_time = util::GpsToUnixSeconds(imu_msg.measurement_time());
  imu_rfu->measurement_time = measurement_time;
  imu_rfu->fb[0] = imu_msg.linear_acceleration().x();
  imu_rfu->fb[1] = imu_msg.linear_acceleration().y();
  imu_rfu->fb[2] = imu_msg.linear_acceleration().z();

  imu_rfu->wibb[0] = imu_msg.angular_velocity().x();
  imu_rfu->wibb[1] = imu_msg.angular_velocity().y();
  imu_rfu->wibb[2] = imu_msg.angular_velocity().z();
  return;
}

void LocalizationInteg::TransferImuFlu(const drivers::gnss::Imu &imu_msg,
                                       ImuData *imu_flu) {
  double measurement_time = util::GpsToUnixSeconds(imu_msg.measurement_time());
  imu_flu->measurement_time = measurement_time;
  imu_flu->fb[0] = -imu_msg.linear_acceleration().y();
  imu_flu->fb[1] = imu_msg.linear_acceleration().x();
  imu_flu->fb[2] = imu_msg.linear_acceleration().z();

  imu_flu->wibb[0] = -imu_msg.angular_velocity().y();
  imu_flu->wibb[1] = imu_msg.angular_velocity().x();
  imu_flu->wibb[2] = imu_msg.angular_velocity().z();
  return;
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
