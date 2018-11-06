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

#include <cmath>

#include "modules/common/math/math_utils.h"
#include "modules/prediction/common/feature_output.h"
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

bool IsClosed(const double x0, const double y0, const double theta0,
              const double x1, const double y1, const double theta1) {
  // TODO(all) move constants to gflags
  double angle_diff = std::abs(common::math::AngleDiff(theta0, theta1));
  double distance = std::hypot(x0 - x1, y0 - y1);
  return distance < 1.0 && angle_diff < M_PI * 0.25;
}

JunctionMLPEvaluator::JunctionMLPEvaluator() {
  LoadModel(FLAGS_evaluator_vehicle_junction_mlp_file);
}

void JunctionMLPEvaluator::Clear() {
}

void JunctionMLPEvaluator::Evaluate(Obstacle* obstacle_ptr) {
  // Sanity checks.
  Clear();
  CHECK_NOTNULL(obstacle_ptr);
  int id = obstacle_ptr->id();
  if (!obstacle_ptr->latest_feature().IsInitialized()) {
    AERROR << "Obstacle [" << id << "] has no latest feature.";
    return;
  }

  JunctionExit curr_junction_exit;
  bool is_closed_to_junction_exit =
      IsClosedToJunctionExit(obstacle_ptr, &curr_junction_exit);

  if (is_closed_to_junction_exit) {
    // TODO(all) Predict based on curr_junction_exit
    return;
  }

  if (obstacle_ptr->history_size() == 0 ||
      !obstacle_ptr->latest_feature().has_junction_feature() ||
      obstacle_ptr->latest_feature().junction_feature()
                                    .junction_exit_size() <= 1) {
    // TODO(all) Predict based on only one exit
    return;
  }

  std::vector<double> feature_values;

  ExtractFeatureValues(obstacle_ptr, &feature_values);

  if (FLAGS_prediction_offline_mode) {
    FeatureOutput::Insert(obstacle_ptr->latest_feature());
    ADEBUG << "Insert junction feature into feature output";
  }
}

bool JunctionMLPEvaluator::IsClosedToJunctionExit(
    const Obstacle* obstacle_ptr,
    JunctionExit* const curr_junction_exit) {
  if (obstacle_ptr == nullptr ||
      obstacle_ptr->history_size() == 0) {
    AERROR << "Null obstacle or no history found";
    return false;
  }
  const Feature& latest_feature = obstacle_ptr->latest_feature();
  double position_x = latest_feature.position().x();
  double position_y = latest_feature.position().y();
  double raw_velocity_heading = std::atan2(latest_feature.raw_velocity().y(),
                                           latest_feature.raw_velocity().x());

  for (const JunctionExit& junction_exit :
       latest_feature.junction_feature().junction_exit()) {
    double exit_x = junction_exit.exit_position().x();
    double exit_y = junction_exit.exit_position().y();
    double exit_heading = junction_exit.exit_heading();
    if (IsClosed(position_x, position_y, raw_velocity_heading,
                 exit_x, exit_y, exit_heading)) {
      *curr_junction_exit = junction_exit;
      return true;
    }
  }

  return false;
}

void JunctionMLPEvaluator::ExtractFeatureValues(
    Obstacle* obstacle_ptr, std::vector<double>* feature_values) {
  CHECK_NOTNULL(obstacle_ptr);
  int id = obstacle_ptr->id();

  std::vector<double> obstacle_feature_values;
  SetObstacleFeatureValues(obstacle_ptr, &obstacle_feature_values);

  if (obstacle_feature_values.size() != OBSTACLE_FEATURE_SIZE) {
    AERROR << "Obstacle [" << id << "] has fewer than "
           << "expected obstacle feature_values "
           << obstacle_feature_values.size() << ".";
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
                         obstacle_feature_values.begin(),
                         obstacle_feature_values.end());
  feature_values->insert(feature_values->end(),
                         junction_feature_values.begin(),
                         junction_feature_values.end());
  if (FLAGS_prediction_offline_mode) {
    SaveOfflineFeatures(obstacle_ptr->mutable_latest_feature(),
                        *feature_values);
    ADEBUG << "Save junction mlp features for obstacle ["
           << obstacle_ptr->id() << "] with dim ["
           << feature_values->size() << "]";
  }
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
  Feature* feature_ptr = obstacle_ptr->mutable_latest_feature();
  if (!feature_ptr->has_position()) {
    ADEBUG << "Obstacle [" << obstacle_ptr->id() << "] has no position.";
    return;
  }
  double heading = std::atan2(feature_ptr->raw_velocity().y(),
                              feature_ptr->raw_velocity().x());
  if (!feature_ptr->has_junction_feature()) {
    AERROR << "Obstacle [" << obstacle_ptr->id()
           << "] has no junction_feature.";
    return;
  }
  std::string junction_id = feature_ptr->junction_feature().junction_id();
  double junction_range = feature_ptr->junction_feature().junction_range();
  for (int i = 0; i < 12; ++i) {
    feature_values->push_back(0.0);
    feature_values->push_back(1.0);
    feature_values->push_back(1.0);
    feature_values->push_back(1.0);
    feature_values->push_back(0.0);
  }
  int num_junction_exit = feature_ptr->junction_feature().junction_exit_size();
  for (int i = 0; i < num_junction_exit; ++i) {
    const JunctionExit& junction_exit =
        feature_ptr->junction_feature().junction_exit(i);
    double x = junction_exit.exit_position().x() - feature_ptr->position().x();
    double y = junction_exit.exit_position().y() - feature_ptr->position().y();
    double diff_x = std::cos(-heading) * x - std::sin(-heading) * y;
    double diff_y = std::sin(-heading) * x + std::cos(-heading) * y;
    double diff_heading = apollo::common::math::AngleDiff(heading,
                              junction_exit.exit_heading());
    double angle = std::atan2(diff_y, diff_x);
    // TODO(Hongyi) test d_idx
    double d_idx = (angle / (2.0 * M_PI)) * 12.0;
    int idx = static_cast<int>(floor(d_idx >= 0 ? d_idx : d_idx + 12));
    feature_values->operator[](idx * 5) = 1.0;
    feature_values->operator[](idx * 5 + 1) = diff_x / junction_range;
    feature_values->operator[](idx * 5 + 2) = diff_y / junction_range;
    feature_values->operator[](idx * 5 + 3) =
        std::sqrt(diff_x * diff_x + diff_y * diff_y) / junction_range;
    feature_values->operator[](idx * 5 + 4) = diff_heading;
  }
}

void JunctionMLPEvaluator::SaveOfflineFeatures(
    Feature* feature_ptr,
    const std::vector<double>& feature_values) {
  for (double feature_value : feature_values) {
    feature_ptr->mutable_junction_feature()
               ->add_junction_mlp_feature(feature_value);
  }
}

void JunctionMLPEvaluator::LoadModel(const std::string& model_file) {
  model_ptr_.reset(new FnnVehicleModel());
  CHECK(model_ptr_ != nullptr);
  CHECK(common::util::GetProtoFromFile(model_file, model_ptr_.get()))
      << "Unable to load model file: " << model_file << ".";

  AINFO << "Succeeded in loading the model file: " << model_file << ".";
}

double JunctionMLPEvaluator::ComputeProbability(
    const std::vector<double>& feature_values) {
  // TODO(all) implement
  return 0.5;
}

}  // namespace prediction
}  // namespace apollo
