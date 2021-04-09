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
 * @file predictor_print_error.h
 * @brief The class of PredictorPrintError.
 */

#ifndef MODULES_LOCALIZATION_LMD_PREDICTOR_OUTPUT_PREDICTOR_PRINT_ERROR_H_
#define MODULES_LOCALIZATION_LMD_PREDICTOR_OUTPUT_PREDICTOR_PRINT_ERROR_H_

#include "modules/localization/lmd/predictor/predictor.h"

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {

/**
 * @class PredictorPrintError
 *
 * @brief  Implementation of predictor.
 */
class PredictorPrintError : public Predictor {
 public:
  explicit PredictorPrintError(double memory_cycle_sec);
  virtual ~PredictorPrintError();

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
  double DepsTimestamp() const;
  void PrintPoseError(double timestamp_sec, const Pose &pose);

 private:
  bool has_latest_timestamp_sec_ = false;
  double latest_timestamp_sec_;
};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_LMD_PREDICTOR_OUTPUT_PREDICTOR_PRINT_ERROR_H_
