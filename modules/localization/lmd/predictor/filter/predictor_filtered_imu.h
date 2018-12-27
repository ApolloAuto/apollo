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
 * @file predictor_filtered_imu.h
 * @brief The class of PredictorFilteredImu.
 */

#ifndef MODULES_LOCALIZATION_LMD_PREDICTOR_FILTER_PREDICTOR_FILTERED_IMU_H_
#define MODULES_LOCALIZATION_LMD_PREDICTOR_FILTER_PREDICTOR_FILTERED_IMU_H_

#include "modules/canbus/proto/chassis.pb.h"

#include "modules/common/math/kalman_filter.h"
#include "modules/localization/lmd/predictor/predictor.h"

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {

/**
 * @class PredictorImu
 *
 * @brief  Implementation of predictor.
 */
class PredictorFilteredImu : public Predictor {
 public:
  explicit PredictorFilteredImu(double memory_cycle_sec);
  virtual ~PredictorFilteredImu();

  /**
   * @brief Update info of chassis.
   * @param imu The message from chassis.
   * @return True if success; false if not needed.
   */
  bool UpdateChassis(const apollo::canbus::Chassis& chassis);

  /**
   * @brief Overrided implementation of the virtual function "Updateable" in the
   * base class "Predictor".
   * @return True if yes; no otherwise.
   */
  bool Updateable() const override;

  /**
   * @brief Overrided implementation of the virtual function "Update" in the
   * base class "Predictor".
   * @return Status::OK() if success; error otherwise.
   */
  apollo::common::Status Update() override;

 private:
  void AccelKalmanFilterInit(double a_estimate);
  double AccelKalmanFilterProcess(double a_control, double a_observation);

 private:
  PoseList chassis_speed_;
  apollo::common::math::KalmanFilter<double, 1, 1, 1> accel_kf_;
};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_LMD_PREDICTOR_FILTER_PREDICTOR_FILTERED_IMU_H_
