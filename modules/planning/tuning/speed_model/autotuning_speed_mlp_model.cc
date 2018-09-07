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
#include "modules/planning/tuning/speed_model/autotuning_speed_mlp_model.h"
#include "modules/planning/tuning/speed_model/autotuning_speed_feature_builder.h"

namespace apollo {
namespace planning {

common::Status AutotuningSpeedMLPModel::SetParams() {
  mlp_model_.reset(new AutotuningMLPModel());
  feature_builder_.reset(new AutotuningSpeedFeatureBuilder());
  return common::Status::OK();
}

double AutotuningSpeedMLPModel::Evaluate(
    const autotuning::TrajectoryFeature& trajectory_feature) const {
  return 0.0;
}

double AutotuningSpeedMLPModel::Evaluate(
    const autotuning::TrajectoryPointwiseFeature& point_feature) const {
  return 0.0;
}

}  // namespace planning
}  // namespace apollo
