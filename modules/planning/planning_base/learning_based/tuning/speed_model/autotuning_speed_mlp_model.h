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

#pragma once

#include "modules/common/status/status.h"
#include "modules/planning/planning_base/learning_based/tuning/autotuning_base_model.h"

namespace apollo {
namespace planning {

class AutotuningSpeedMLPModel : public AutotuningBaseModel {
 public:
  AutotuningSpeedMLPModel() = default;
  ~AutotuningSpeedMLPModel() = default;

  /**
   * @brief set mlp model as well as feature builder
   */
  common::Status SetParams() override;

  /**
   * @brief : evaluate by trajectory
   * @param : input trajectory feature proto
   * @return : the total value of reward / cost
   */
  double Evaluate(
      const autotuning::TrajectoryFeature& trajectory_feature) const override;

  /**
   * @brief: evaluate by trajectory point
   * @param : trajectory pointwise input feature
   * @return : total value of reward / cost
   */
  double Evaluate(const autotuning::TrajectoryPointwiseFeature& point_feature)
      const override;

 private:
  /**
   * [FlattenFeatures description]
   * @param feature      [model input proto]
   * @param flat_feature [eigen matrix, row: time, col: traj point feature]
   */
  void FlattenFeatures(const autotuning::TrajectoryFeature& feature,
                       Eigen::MatrixXd* const flat_feature) const;

  void FlattenFeatures(
      const autotuning::SpeedPointwiseFeature& speed_point_feature,
      const int row, Eigen::MatrixXd* const flat_feature) const;
};

}  // namespace planning
}  // namespace apollo
