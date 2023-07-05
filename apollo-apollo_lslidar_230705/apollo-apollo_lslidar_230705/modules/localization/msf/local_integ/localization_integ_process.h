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

#include <queue>
#include <string>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cyber/cyber.h"

#include "localization_msf/sins.h"
#include "modules/common/status/status.h"
#include "modules/localization/msf/local_integ/localization_params.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"

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
  apollo::common::Status Init(const LocalizationIntegParam &params);

  // Raw Imu process.
  void RawImuProcess(const ImuData &imu_msg);
  void GetState(IntegState *state);
  void GetResult(IntegState *state, LocalizationEstimate *localization);
  void GetResult(IntegState *state, InsPva *sins_pva,
                 double pva_covariance[9][9]);
  void GetCorrectedImu(ImuData *imu_data);
  void GetEarthParameter(InertialParameter *earth_param);

  // itegration measure data process
  void MeasureDataProcess(const MeasureData &measure_msg);

 private:
  bool CheckIntegMeasureData(const MeasureData &measure_data);

  bool LoadGnssAntennaExtrinsic(const std::string &file_path,
                                TransformD *extrinsic) const;

  void MeasureDataProcessImpl(const MeasureData &measure_msg);
  void MeasureDataThreadLoop();
  void StartThreadLoop();
  void StopThreadLoop();

  void GetValidFromOK();

 private:
  Sins *sins_;

  // config
  TransformD gnss_antenna_extrinsic_;

  // double imu_rate_;

  IntegState integ_state_;
  InsPva ins_pva_;
  double pva_covariance_[9][9];

  ImuData corrected_imu_;
  InertialParameter earth_param_;

  std::atomic<bool> keep_running_;
  std::queue<MeasureData> measure_data_queue_;
  int measure_data_queue_size_ = 150;
  std::mutex measure_data_queue_mutex_;

  int delay_output_counter_ = 0;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
