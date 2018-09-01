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

#include "modules/planning/tuning/autotuning_raw_feature_generator.h"

namespace apollo {
namespace planning {

AutotuningRawFeatureGenerator::AutotuningRawFeatureGenerator(
    const std::vector<double>& evaluate_time) {
  evaluate_time_ = evaluate_time;
}

common::Status AutotuningRawFeatureGenerator::evaluate_trajectory(
    const std::vector<common::TrajectoryPoint>& trajectory,
    const ReferenceLineInfo& reference_line_info, const Frame& frame,
    autotuning::TrajectoryRawFeature* const trajectory_feature) const {
  return common::Status::OK();
}

common::Status AutotuningRawFeatureGenerator::evaluate_trajectory_point(
    const common::TrajectoryPoint& trajectory_point,
    const ReferenceLineInfo& reference_line_info, const Frame& frame,
    autotuning::TrajectoryPointRawFeature* const trajectory_point_feature)
    const {
  return common::Status::OK();
}

}  // namespace planning
}  // namespace apollo
