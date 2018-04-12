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

#ifndef MODULES_LOCALIZATION_MSF_LOCALIZATION_IMU_PROCESS_H_
#define MODULES_LOCALIZATION_MSF_LOCALIZATION_IMU_PROCESS_H_

#include <list>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <atomic>
#include <condition_variable>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "modules/common/status/status.h"
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

enum class IntegState { NOT_INIT = 0, NOT_STABLE, OK, VALID };

/**
 * @class LocalizationIntegProcess
 *
 * @brief process Imu msg for localization
 */

class LocalizationIntegProcess {
 public:
  typedef Eigen::Affine3d TransformD;

  LocalizationIntegProcess();
  ~LocalizationIntegProcess();

  // Initialization.
  apollo::common::Status Init(const LocalizationIntegParam& params);

  // Raw Imu process.
  void RawImuProcess(const ImuData &imu_msg);
  void GetState(IntegState *state);
  void GetResult(IntegState *state, LocalizationEstimate *localization);
  void GetResult(IntegState *state,
                 InsPva *sins_pva,
                 double pva_covariance[9][9]);

  // itegration measure data process
  void MeasureDataProcess(const MeasureData& measure_msg);

 private:
  bool CheckIntegMeasureData(const MeasureData& measure_data);

  bool LoadGnssAntennaExtrinsic(const std::string &file_path,
                                TransformD *extrinsic) const;

  void MeasureDataProcessImpl(const MeasureData& measure_msg);
  void MeasureDataThreadLoop();
  void StartThreadLoop();
  void StopThreadLoop();

  void GetValidFromOK();

 private:
  Sins *sins_;

  // config
  TransformD gnss_antenna_extrinsic_;

  bool debug_log_flag_;
  // double imu_rate_;

  IntegState integ_state_;
  InsPva ins_pva_;
  double pva_covariance_[9][9];

  std::atomic<bool> keep_running_;
  std::thread measure_data_thread_;
  std::condition_variable new_measure_data_signal_;
  std::queue<MeasureData> measure_data_queue_;
  int measure_data_queue_size_;
  std::mutex measure_data_queue_mutex_;

  int delay_output_counter_;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_MSF_LOCALIZATION_IMU_PROCESS_H_
