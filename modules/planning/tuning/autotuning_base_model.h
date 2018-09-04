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

#ifndef MODULES_PLANNING_TUNING_AUTOTUNING_BASE_MODEL_H_
#define MODULES_PLANNING_TUNING_AUTOTUNING_BASE_MODEL_H_

#include <memory>

namespace apollo {
namespace planning {

class AutotuningBaseModel {
 public:
  /**
   * @brief constructor
   */
  AutotuningBaseModel() = default;

  /**
   * @brief destructor
   */
  ~AutotuningBaseModel() = default;

  /**
   * @brief : evaluate by trajectory
   * @param : input trajectory feature proto
   * @return : the total value of reward / cost
   */
  double Evaluate(
      const autotuning::TrajectoryFeature& trajectory_feature) const = 0;

  /**
   * @brief: evaluate by trajectory point
   * @param : trajectory pointwise input feature
   * @return : total value of reward / cost
   */
  double Evaluate(
      const autotuning::TrajectoryPointwiseFeature& point_feature) const = 0;

 private:
  /**
   * @brief : stored autotuning mlp model
   */
  std::unique_ptr<AutotuningMLPNetModel> mpl_model_ = nullptr;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TUNING_AUTOTUNING_BASE_MODEL_H_
