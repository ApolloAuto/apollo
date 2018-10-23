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

void JunctionMLPEvaluator::ExtractFeatureValues(
    Obstacle* obstacle_ptr, std::vector<double>* feature_values) {
  CHECK_NOTNULL(obstacle_ptr);
  int id = obstacle_ptr->id();

  auto it = obstacle_feature_values_map_.find(id);
  if (it == obstacle_feature_values_map_.end()) {
    std::vector<double> obstacle_feature_values;
    SetObstacleFeatureValues(obstacle_ptr, &obstacle_feature_values);
    obstacle_feature_values_map_[id] = obstacle_feature_values;
  }

  if (obstacle_feature_values_map_[id].size() != OBSTACLE_FEATURE_SIZE) {
    ADEBUG << "Obstacle [" << id << "] has fewer than "
           << "expected obstacle feature_values "
           << obstacle_feature_values_map_[id].size() << ".";
    return;
  }

  std::vector<double> junction_feature_values;
  SetJunctionFeatureValues(obstacle_ptr, &junction_feature_values);
  if (junction_feature_values.size() != JUNCTION_FEATURE_SIZE) {
    ADEBUG << "Obstacle [" << id << "] has fewer than "
           << "expected junction feature_values"
           << junction_feature_values.size() << ".";
    return;
  }

  feature_values->insert(feature_values->end(),
                         obstacle_feature_values_map_[id].begin(),
                         obstacle_feature_values_map_[id].end());
  feature_values->insert(feature_values->end(), junction_feature_values.begin(),
                         junction_feature_values.end());
}

void JunctionMLPEvaluator::SetObstacleFeatureValues(
    Obstacle* obstacle_ptr, std::vector<double>* feature_values) {
  feature_values->clear();
  feature_values->reserve(OBSTACLE_FEATURE_SIZE);
  const Feature& feature = obstacle_ptr->latest_feature();
  if (!feature.has_position()) {
    ADEBUG << "Obstacle [" << obstacle_ptr->id() << "] has no position.";
    return;
  }
  feature_values->push_back(feature.speed());
  feature_values->push_back(feature.acc());
  feature_values->push_back(feature.junction_feature().junction_range());
}

void JunctionMLPEvaluator::SetJunctionFeatureValues(
    Obstacle* obstacle_ptr, std::vector<double>* feature_values) {
  feature_values->clear();
  feature_values->reserve(JUNCTION_FEATURE_SIZE);
  const Feature& feature = obstacle_ptr->latest_feature();
  if (!feature.has_position()) {
    ADEBUG << "Obstacle [" << obstacle_ptr->id() << "] has no position.";
    return;
  }
  double heading = feature.velocity_heading();
  if (!feature.has_junction_feature()) {
    AERROR << "Obstacle [" << obstacle_ptr->id()
           << "] has no junction_feature.";
    return;
  }
  std::string junction_id = feature.junction_feature().junction_id();
  double junction_range = feature.junction_feature().junction_range();
  for (int i = 0; i < 12; ++i) {
    feature_values->push_back(0);
    feature_values->push_back(1);
    feature_values->push_back(1);
    feature_values->push_back(1);
    feature_values->push_back(0);
  }
  int num_junction_exit = feature.junction_feature().junction_exit_size();
  for (int i = 0; i < num_junction_exit; ++i) {
    const JunctionExit& junction_exit =
        feature.junction_feature().junction_exit(i);
    double x = junction_exit.exit_position().x() - feature.position().x();
    double y = junction_exit.exit_position().y() - feature.position().y();
    double diff_x = std::cos(heading) * x - std::sin(heading) * y;
    double diff_y = std::sin(heading) * x + std::cos(heading) * y;
    double angle = std::atan2(diff_x, diff_y);
    double d_idx = (angle / (2.0 * std::acos(-1))) -
        floor((angle / (2.0 * std::acos(-1.0))) / 12.0) * 12.0;
    int idx = static_cast<int>(d_idx >= 0 ? d_idx : d_idx + 12);
    feature_values->operator[](idx * 5) = 1;
    feature_values->operator[](idx * 5 + 1) = diff_x / junction_range;
    feature_values->operator[](idx * 5 + 2) = diff_y / junction_range;
    feature_values->operator[](idx * 5 + 3) =
        std::sqrt(diff_x * diff_x + diff_y * diff_y) / junction_range;
    feature_values->operator[](idx * 5 + 4) =
        junction_exit.exit_heading() - heading;
  }
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
