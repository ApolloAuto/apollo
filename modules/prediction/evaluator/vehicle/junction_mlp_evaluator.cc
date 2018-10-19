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

#include "modules/prediction/evaluator/vehicle/junction_mlp_evaluator.h"

#include <memory>

#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_map.h"

using apollo::hdmap::LaneInfo;

namespace apollo {
namespace prediction {

double ComputeMean(const std::vector<double>& nums, size_t start, size_t end) {
  int count = 0;
  double sum = 0.0;
  for (size_t i = start; i <= end && i < nums.size(); i++) {
    sum += nums[i];
    ++count;
  }
  return (count == 0) ? 0.0 : sum / count;
}

JunctionMLPEvaluator::JunctionMLPEvaluator() {
  LoadModel(FLAGS_evaluator_vehicle_junction_mlp_file);
}

void JunctionMLPEvaluator::Clear() {
  obstacle_feature_values_map_.clear();
}

void JunctionMLPEvaluator::Evaluate(Obstacle* obstacle_ptr) {
  // TODO(all) implement
  // 1. extract features
  // 2. compute probabilities
}

void JunctionMLPEvaluator::SetObstacleFeatureValues(
    Obstacle* obstacle_ptr, std::vector<double>* feature_values) {
  feature_values->clear();
  feature_values->reserve(OBSTACLE_FEATURE_SIZE);
  // TODO(all) implement
}

void JunctionMLPEvaluator::SetJunctionFeatureValues(
    Obstacle* obstacle_ptr, std::vector<double>* feature_values) {
  feature_values->clear();
  feature_values->reserve(JUNCTION_FEATURE_SIZE);
  // TODO(all) implement
}

void JunctionMLPEvaluator::FindJunctionPath(
    Obstacle* obstacle_ptr, std::vector<double>* path_values) {
  CHECK_NOTNULL(obstacle_ptr);
  path_values->clear();
  path_values->reserve(40);
  const Feature& feature = obstacle_ptr->latest_feature();
  if (!feature.has_junction_feature()) {
    AERROR << "Obstacle [" << obstacle_ptr->id()
           << "] has no junction_feature.";
    return;
  }
  std::string junction_id = "";
  if (feature.junction_feature().has_junction_id()) {
    junction_id = feature.junction_feature().junction_id();
  }
  int num_junction_exit = feature.junction_feature().junction_exit_size();
  for (int i = 0; i < num_junction_exit; ++i) {
    const JunctionExit& junction_exit =
        feature.junction_feature().junction_exit(i);
    if (!junction_exit.has_pred_exit_lane_id()) {
      AERROR << "JunctionExit [" << i
             << "] has no pred_exit_lane.";
      return;
    }
    const std::string& pred_exit_lane_id = junction_exit.pred_exit_lane_id();
    std::shared_ptr<const LaneInfo> pre_exit_lane_info =
        PredictionMap::LaneById(pred_exit_lane_id);
    // Under construction~
  }
  // TODO(all) implement
}

void JunctionMLPEvaluator::LoadModel(const std::string& model_file) {
  // TODO(all) implement
  // 1. Make model file ready
  // 2. Load model from file
}

double JunctionMLPEvaluator::ComputeProbability(
    const std::vector<double>& feature_values) {
  // TODO(all) implement
  return 0.5;
}

}  // namespace prediction
}  // namespace apollo
