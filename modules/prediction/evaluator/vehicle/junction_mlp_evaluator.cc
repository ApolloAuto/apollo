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

#include <unordered_map>

#include "modules/prediction/common/feature_output.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_map.h"
#include "modules/prediction/common/prediction_system_gflags.h"
#include "modules/prediction/common/prediction_util.h"

namespace apollo {
namespace prediction {

namespace {

double ComputeMean(const std::vector<double>& nums, size_t start, size_t end) {
  int count = 0;
  double sum = 0.0;
  for (size_t i = start; i <= end && i < nums.size(); i++) {
    sum += nums[i];
    ++count;
  }
  return (count == 0) ? 0.0 : sum / count;
}

}  // namespace

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
  Feature* latest_feature_ptr = obstacle_ptr->mutable_latest_feature();
  CHECK_NOTNULL(latest_feature_ptr);

  // Assume obstacle is NOT closed to any junction exit
  if (!latest_feature_ptr->has_junction_feature() ||
      latest_feature_ptr->junction_feature().junction_exit_size() < 1) {
    ADEBUG << "Obstacle [" << id << "] has no junction_exit.";
    return;
  }

  std::vector<double> feature_values;
  ExtractFeatureValues(obstacle_ptr, &feature_values);
  std::vector<double> probability;
  if (latest_feature_ptr->junction_feature().junction_exit_size() > 1) {
    probability = ComputeProbability(feature_values);
  } else {
    for (int i = 0; i < 12; ++i) {
      probability.push_back(feature_values[3 + 5 * i]);
    }
  }
  for (double prob : probability) {
    latest_feature_ptr->mutable_junction_feature()
                      ->add_junction_mlp_probability(prob);
  }

  // assign all lane_sequence probability
  LaneGraph* lane_graph_ptr =
      latest_feature_ptr->mutable_lane()->mutable_lane_graph();
  CHECK_NOTNULL(lane_graph_ptr);
  if (lane_graph_ptr->lane_sequence_size() == 0) {
    AERROR << "Obstacle [" << id << "] has no lane sequences.";
    return;
  }

  std::unordered_map<std::string, double> junction_exit_prob;
  for (const JunctionExit& junction_exit :
       latest_feature_ptr->junction_feature().junction_exit()) {
    double x = junction_exit.exit_position().x()
             - latest_feature_ptr->position().x();
    double y = junction_exit.exit_position().y()
             - latest_feature_ptr->position().y();
    double angle = std::atan2(y, x)
                 - std::atan2(latest_feature_ptr->raw_velocity().y(),
                              latest_feature_ptr->raw_velocity().x());
    double d_idx = (angle / (2.0 * M_PI)) * 12.0;
    int idx = static_cast<int>(floor(d_idx >= 0 ? d_idx : d_idx + 12));
    int prev_idx = idx == 0 ? 11 : idx - 1;
    int post_idx = idx == 11 ? 0 : idx + 1;
    junction_exit_prob[junction_exit.exit_lane_id()] = probability[idx] * 0.5
        + probability[prev_idx] * 0.25 + probability[post_idx] * 0.25;
  }

  for (int i = 0; i < lane_graph_ptr->lane_sequence_size(); ++i) {
    LaneSequence* lane_sequence_ptr =
        lane_graph_ptr->mutable_lane_sequence(i);
    CHECK_NOTNULL(lane_sequence_ptr);
    for (const LaneSegment& lane_segment :
         lane_sequence_ptr->lane_segment()) {
      if (junction_exit_prob.find(lane_segment.lane_id()) !=
          junction_exit_prob.end()) {
        lane_sequence_ptr->set_probability(
          junction_exit_prob[lane_segment.lane_id()]);
      }
    }
  }
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
    AERROR << "Obstacle [" << id << "] has fewer than "
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

std::vector<double> JunctionMLPEvaluator::ComputeProbability(
    const std::vector<double>& feature_values) {
  CHECK_NOTNULL(model_ptr_.get());
  if (model_ptr_->dim_input() != static_cast<int>(feature_values.size())) {
    ADEBUG << "Model feature size not consistent with model proto definition. "
           << "model input dim = " << model_ptr_->dim_input()
           << "; feature value size = " << feature_values.size();
    return {};
  }
  std::vector<double> layer_input;
  layer_input.reserve(model_ptr_->dim_input());
  std::vector<double> layer_output;
  for (int i = 0; i < model_ptr_->num_layer(); ++i) {
    if (i > 0) {
      layer_input.swap(layer_output);
      layer_output.clear();
    }
    const Layer& layer = model_ptr_->layer(i);
    for (int col = 0; col < layer.layer_input_weight().rows(0).columns_size();
         ++col) {
      double neuron_output = layer.layer_bias().columns(col);
      for (int row = 0; row < layer.layer_input_weight().rows_size(); ++row) {
        double weight = layer.layer_input_weight().rows(row).columns(col);
        neuron_output += (layer_input[row] * weight);
      }
      if (layer.layer_activation_func() == Layer::RELU) {
        neuron_output = apollo::prediction::math_util::Relu(neuron_output);
      } else if (layer.layer_activation_func() == Layer::SIGMOID) {
        neuron_output = apollo::prediction::math_util::Sigmoid(neuron_output);
      } else if (layer.layer_activation_func() == Layer::TANH) {
        neuron_output = std::tanh(neuron_output);
      }
      layer_output.push_back(neuron_output);
    }
    if (layer.layer_activation_func() == Layer::SOFTMAX) {
      layer_output = apollo::prediction::math_util::Softmax(
          layer_output, false);
    }
  }
  return layer_output;
}

}  // namespace prediction
}  // namespace apollo
