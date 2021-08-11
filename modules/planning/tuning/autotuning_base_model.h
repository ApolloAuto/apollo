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

#include <memory>

#include "modules/common/status/status.h"
#include "modules/planning/proto/auto_tuning_model_input.pb.h"
#include "modules/planning/tuning/autotuning_feature_builder.h"
#include "modules/planning/tuning/autotuning_mlp_net_model.h"

namespace apollo {
namespace planning {

class AutotuningBaseModel {
 public:
  AutotuningBaseModel() = default;
  virtual ~AutotuningBaseModel() = default;

  /**
   * @brief set mlp model as well as feature builder
   */
  virtual common::Status SetParams() = 0;

  /**
   * @brief : evaluate by trajectory
   * @param : input trajectory feature proto
   * @return : the total value of reward / cost
   */
  virtual double Evaluate(
      const autotuning::TrajectoryFeature& trajectory_feature) const = 0;

  /**
   * @brief: evaluate by trajectory point
   * @param : trajectory pointwise input feature
   * @return : total value of reward / cost
   */
  virtual double Evaluate(
      const autotuning::TrajectoryPointwiseFeature& point_feature) const = 0;

 protected:
  /**
   * @brief : stored autotuning mlp model
   */
  std::unique_ptr<AutotuningMLPModel> mlp_model_ = nullptr;
  std::unique_ptr<AutotuningFeatureBuilder> feature_builder_ = nullptr;
};

}  // namespace planning
}  // namespace apollo
