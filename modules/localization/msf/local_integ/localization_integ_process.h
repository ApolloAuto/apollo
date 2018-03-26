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

#ifndef MODULES_LOCALIZATION_MSF_LOCALIZATION_IMU_PROCESS_H_
#define MODULES_LOCALIZATION_MSF_LOCALIZATION_IMU_PROCESS_H_

#include <pthread.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <condition_variable>
#include <list>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <atomic>
#include "modules/localization/msf/local_integ/localization_params.h"
#include "modules/localization/proto/localization.pb.h"
#include "include/sins.h"

/**
 * @namespace apollo::localization::msf
 * @brief apollo::localization::msf
 */
namespace apollo {
namespace localization {
namespace msf {

typedef Eigen::Affine3d TransformD;
typedef Eigen::Vector3d Vector3D;
typedef Eigen::Translation3d Translation3D;
typedef Eigen::Matrix3d Matrix3D;
typedef Eigen::Quaterniond QuaternionD;

enum class IntegState { NOT_INIT = 0, NOT_STABLE, OK, VALID };

/**
 * @class LocalizationIntegProcess
 *
 * @brief process Imu msg for localization
 */

class LocalizationIntegProcess {
 public:
  LocalizationIntegProcess();
  ~LocalizationIntegProcess();
  // Initialization.
  LocalizationState Init(const LocalizationIntegParam& params);

  // Raw Imu process.
  void RawImuProcess(const ImuData &imu_msg);
  void GetState(IntegState *state);
  void GetResult(IntegState *state, InsPva *sins_pva,
                 LocalizationEstimate *localization);
  void GetResult(MeasureData *measure_data);

  // itegration measure data process
  void MeasureDataProcess(const MeasureData& measure_msg);

 private:
  bool CheckIntegMeasureData(const MeasureData& measure_data);

  bool LoadGnssAntennaExtrinsic(std::string file_path,
                                TransformD *extrinsic) const;

  void MeasureDataProcessImpl(const MeasureData& measure_msg);
  void MeasureDataThreadLoop();

  void GetValidFromOK();

 private:
  Sins *sins_;

  // // integration
  // IntegratedNavigation integ_nav_sins_update_;
  // IntegratedNavigation integ_nav_measure_update_;
  // pthread_mutex_t integ_time_update_mutex_;

  // // imu msg
  // std::list<SinsImuData> rawimu_list_;
  // std::list<SinsImuData> rawimu_list_measure_update_;
  // pthread_mutex_t imu_mutex_;

  // // measure data
  // std::list<MeasureData> measure_data_list_;
  // pthread_mutex_t measure_callback_mutex_;

  // config
  TransformD gnss_antenna_extrinsic_;
  // TransformD velodyne_extrinsic_;
  // TransformD wheelspeed_extrinsic_;

  // bool is_sins_align_with_vel_;
  // double vel_threshold_get_yaw_;

  bool debug_log_flag_;
  double imu_rate_;

  // // temporary variable
  // double pre_measure_update_time_;
  // bool measure_callback_execute_;

  // bool is_using_raw_gnsspos_;

  IntegState integ_state_;
  InsPva ins_pva_;
  double pva_covariance_[9][9];

  std::atomic<bool> keep_running_;
  std::thread measure_data_thread_;
  std::condition_variable new_measure_data_signal_;
  std::queue<MeasureData> measure_data_queue_;
  int measure_data_queue_size_;
  std::mutex measure_data_queue_mutex_;

  // bool is_sins_state_check_;
  // double sins_state_span_time_;
  // double sins_state_pos_std_;
  int delay_output_counter_;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_MSF_LOCALIZATION_IMU_PROCESS_H_
