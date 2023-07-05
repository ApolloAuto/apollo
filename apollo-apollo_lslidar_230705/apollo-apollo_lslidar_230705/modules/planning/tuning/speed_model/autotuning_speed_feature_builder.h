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

#include "modules/planning/tuning/autotuning_feature_builder.h"

namespace apollo {
namespace planning {

/**
 * @brief: build the mlp cost functional for speed profile based on trajectory
 * raw feature
 */
class AutotuningSpeedFeatureBuilder : public AutotuningFeatureBuilder {
 public:
  AutotuningSpeedFeatureBuilder() = default;
  virtual ~AutotuningSpeedFeatureBuilder() = default;
  /**
   * @param: raw feature function input
   * @param: generated model input feature from raw feature, function output
   */
  common::Status BuildFeature(
      const autotuning::TrajectoryRawFeature& raw_feature,
      autotuning::TrajectoryFeature* const input_feature) const override;

  /**
   * @param: pointwise raw feature, function input
   * @param: generated model input feature, function output
   */
  common::Status BuildPointFeature(
      const autotuning::TrajectoryPointRawFeature& raw_point_feature,
      autotuning::TrajectoryPointwiseFeature* const point_feature)
      const override;

 private:
  /**
   * @brief: map virtual stop yield overtake obstacle with feature
   * @param: raw object feature
   * @param: input feature
   */
  void map_obstacle_feature(
      const autotuning::SpeedPointRawFeature_ObjectDecisionFeature&
          obj_raw_feature,
      autotuning::SpeedPointwiseFeature_ObstacleFeature* const input_feature)
      const;

  /**
   * @brief: map nudge obstacle to model input feature
   */
  void map_nudge_obs_feature(
      const autotuning::SpeedPointRawFeature_ObjectDecisionFeature&
          obj_raw_feature,
      autotuning::SpeedPointwiseFeature_ObstacleFeature* const input_feature)
      const;

  /**
   * @brief: map sidepass obstacle to model input feature
   */
  void map_sidepass_obs_feature(
      const autotuning::SpeedPointRawFeature_ObjectDecisionFeature&
          obj_raw_feature,
      autotuning::SpeedPointwiseFeature_ObstacleFeature* const input_feature)
      const;
};

}  // namespace planning
}  // namespace apollo
