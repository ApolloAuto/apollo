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
 * @file predictor.h
 * @brief The class of Predictor.
 */

#ifndef MODULES_LOCALIZATION_LMD_COMMON_PREDICTOR_PREDICTOR_H_
#define MODULES_LOCALIZATION_LMD_COMMON_PREDICTOR_PREDICTOR_H_

#include <map>
#include <string>
#include <utility>

#include "modules/common/log.h"
#include "modules/common/status/status.h"
#include "modules/localization/lmd/common/pose_list.h"

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {

constexpr char kPredictorPrintErrorName[] = "print_error";
constexpr char kPredictorOutputName[] = "output";
constexpr char kPredictorImuName[] = "imu";
constexpr char kPredictorGpsName[] = "gps";
constexpr char kPredictorFilteredImuName[] = "filtered_imu";
constexpr char kPredictorPerceptionName[] = "perception";

/**
 * @class Predictor
 *
 * @brief  Interface for implementing predictor.
 */
class Predictor {
 public:
  explicit Predictor(double memory_cycle_sec_)
      : predicted_(memory_cycle_sec_) {}

  virtual ~Predictor() {}

  /**
   * @brief Get name of the predictor.
   * @return A name.
   */
  const std::string& Name() const { return name_; }

  /**
   * @brief Get names of the deps.
   * @return Predicteds of deps.
   */
  const std::map<std::string, PoseList>& DepPredicteds() const {
    return dep_predicteds_;
  }

  /**
   * @brief Update predicted list of deps.
   */
  void UpdateDepPredicted(const std::string& name, const PoseList& predicted) {
    UpdateDepPredicted(name, PoseList(predicted));
  }

  void UpdateDepPredicted(const std::string& name, PoseList&& predicted) {
    auto it = dep_predicteds_.find(name);
    CHECK(it != dep_predicteds_.end());
    it->second = std::move(predicted);
  }

  /**
   * @brief Get predicted list.
   * @return Predicted list.
   */
  const PoseList& Predicted() const { return predicted_; }

  /**
   * @brief Is predicted running on adapter's thread.
   * @return True if yes; no otherwise.
   */
  bool UpdatingOnAdapterThread() const { return on_adapter_thread_; }

  /**
   * @brief Is predicted list updateable.
   * @return True if yes; no otherwise.
   */
  virtual bool Updateable() const = 0;

  /**
   * @brief Predict a new pose and add it to predicted list.
   * @return Status::OK() if success; error otherwise.
   */
  virtual apollo::common::Status Update() = 0;

 protected:
  std::string name_;

  std::map<std::string, PoseList> dep_predicteds_;
  PoseList predicted_;
  bool on_adapter_thread_ = false;
};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_LMD_COMMON_PREDICTOR_PREDICTOR_H_
