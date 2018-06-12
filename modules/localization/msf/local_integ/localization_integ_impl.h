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

#ifndef MODULES_LOCALIZATION_MSF_LOCALIZATION_INTEG_IMPL_H_
#define MODULES_LOCALIZATION_MSF_LOCALIZATION_INTEG_IMPL_H_

#include <atomic>
#include <functional>
#include <queue>
#include <list>

#include "modules/common/status/status.h"
#include "modules/localization/msf/local_integ/localization_gnss_process.h"
#include "modules/localization/msf/local_integ/localization_integ_process.h"
#include "modules/localization/msf/local_integ/localization_lidar_process.h"
#include "modules/localization/msf/local_integ/measure_republish_process.h"
#include "modules/localization/msf/local_integ/localization_integ.h"

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
  // Raw Imu process.
  void RawImuProcessRfu(const ImuData& imu_data);

  // Gnss Info process.
  void RawObservationProcess(
      const drivers::gnss::EpochObservation& raw_obs_msg);
  void RawEphemerisProcess(const drivers::gnss::GnssEphemeris& gnss_orbit_msg);

  // gnss best pose process
  void GnssBestPoseProcess(const drivers::gnss::GnssBestPose& bestgnsspos_msg);

  void GetLastestLidarLocalization(LocalizationMeasureState *state,
                                   LocalizationEstimate *lidar_localization);

  void GetLastestIntegLocalization(LocalizationMeasureState *state,
                                   LocalizationEstimate *integ_localization);

  void GetLastestGnssLocalization(LocalizationMeasureState *state,
                                  LocalizationEstimate *gnss_localization);

  void GetLidarLocalizationList(std::list<LocalizationResult> *results);

  void GetIntegLocalizationList(std::list<LocalizationResult> *results);

  void GetGnssLocalizationList(std::list<LocalizationResult> *results);

 protected:
  void StartThreadLoop();
  void StopThreadLoop();

  void PcdThreadLoop();
  void PcdProcessImpl(const LidarFrame& pcd_data);

  void ImuThreadLoop();
  void ImuProcessImpl(const ImuData& imu_data);

  void GnssThreadLoop();
  void RawObservationProcessImpl(
      const drivers::gnss::EpochObservation& raw_obs_msg);
  void RawEphemerisProcessImpl(
      const drivers::gnss::GnssEphemeris& gnss_orbit_msg);
  void GnssBestPoseProcessImpl(
      const drivers::gnss::GnssBestPose& bestgnsspos_msg);

  void TransferGnssMeasureToLocalization(const MeasureData& measure,
                                         LocalizationEstimate *localization);

 private:
  MeasureRepublishProcess* republish_process_;
  LocalizationIntegProcess* integ_process_;
  LocalizationGnssProcess* gnss_process_;
  LocalizationLidarProcess* lidar_process_;

  // lidar localizaiton result list
  std::list<LocalizationResult> lidar_localization_list_;
  size_t lidar_localization_list_max_size_;
  std::mutex lidar_localization_mutex_;

  // integration localization result list
  std::list<LocalizationResult> integ_localization_list_;
  size_t integ_localization_list_max_size_;
  std::mutex integ_localization_mutex_;

  // gnss localization result list
  std::list<LocalizationResult> gnss_localization_list_;
  size_t gnss_localization_list_max_size_;
  std::mutex gnss_localization_mutex_;
  bool is_use_gnss_bestpose_;

  // lidar process thread
  std::atomic<bool> keep_lidar_running_;
  std::thread lidar_data_thread_;
  std::condition_variable lidar_data_signal_;
  std::queue<LidarFrame> lidar_data_queue_;
  size_t lidar_queue_max_size_;
  std::mutex lidar_data_queue_mutex_;
  double imu_altitude_from_lidar_localization_;
  bool imu_altitude_from_lidar_localization_available_;

  // imu process thread
  std::atomic<bool> keep_imu_running_;
  std::thread imu_data_thread_;
  std::condition_variable imu_data_signal_;
  std::queue<ImuData> imu_data_queue_;
  size_t imu_queue_max_size_;
  std::mutex imu_data_queue_mutex_;

  // gnss process thread
  std::atomic<bool> keep_gnss_running_;
  std::thread gnss_function_thread_;
  std::condition_variable gnss_function_signal_;
  std::queue<std::function<void()>> gnss_function_queue_;
  size_t gnss_queue_max_size_;
  std::mutex gnss_function_queue_mutex_;

  bool debug_log_flag_;
  bool enable_lidar_localization_;

  Eigen::Affine3d gnss_antenna_extrinsic_;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_MSF_LOCALIZATION_INTEG_IMPL_H_
