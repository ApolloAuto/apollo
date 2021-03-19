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
#include "modules/planning/proto/auto_tuning_model_input.pb.h"
#include "modules/planning/proto/auto_tuning_raw_feature.pb.h"

namespace apollo {
namespace planning {

/**
 * @brief: build model related input feature from raw feature generator
 */

class AutotuningFeatureBuilder {
 public:
  /**
   * @brief: constructor
   */
  AutotuningFeatureBuilder() = default;
  virtual ~AutotuningFeatureBuilder() = default;

  /**
   * @param: raw feature function input
   * @param: generated model input feature from raw feature, function output
   */
  virtual common::Status BuildFeature(
      const autotuning::TrajectoryRawFeature& raw_feature,
      autotuning::TrajectoryFeature* const input_feature) const = 0;

  /**
   * @param: pointwise raw feature, function input
   * @param: generated model input feature, function output
   */
  virtual common::Status BuildPointFeature(
      const autotuning::TrajectoryPointRawFeature& raw_point_feature,
      autotuning::TrajectoryPointwiseFeature* const point_feature) const = 0;
};

}  // namespace planning
}  // namespace apollo
