/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/prediction/evaluator/vehicle/junction_map_evaluator.h"

#include <omp.h>
#include <unordered_map>
#include <utility>

#include "cyber/common/file.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_map.h"
#include "modules/prediction/common/prediction_system_gflags.h"
#include "modules/prediction/common/prediction_util.h"
#include "modules/prediction/common/semantic_map.h"

namespace apollo {
namespace prediction {

JunctionMapEvaluator::JunctionMapEvaluator() : device_(torch::kCPU) {
  evaluator_type_ = ObstacleConf::JUNCTION_MAP_EVALUATOR;
  LoadModel();
}

void JunctionMapEvaluator::Clear() {}

bool JunctionMapEvaluator::Evaluate(Obstacle* obstacle_ptr) {
  // Sanity checks.
  omp_set_num_threads(1);

  obstacle_ptr->SetEvaluatorType(evaluator_type_);

  Clear();
  CHECK_NOTNULL(obstacle_ptr);
  int id = obstacle_ptr->id();
  if (!obstacle_ptr->latest_feature().IsInitialized()) {
    AERROR << "Obstacle [" << id << "] has no latest feature.";
    return false;
  }
  Feature* latest_feature_ptr = obstacle_ptr->mutable_latest_feature();
  CHECK_NOTNULL(latest_feature_ptr);

  // Assume obstacle is NOT closed to any junction exit
  if (!latest_feature_ptr->has_junction_feature() ||
      latest_feature_ptr->junction_feature().junction_exit_size() < 2) {
    ADEBUG << "Obstacle [" << id << "] has less than two junction_exits.";
    return false;
  }
  // Extract features of junction_exit_mask
  std::vector<double> feature_values;
  if (!ExtractFeatureValues(obstacle_ptr, &feature_values)) {
    ADEBUG << "Obstacle [" << id << "] failed to extract junction exit mask";
    return false;
  }

  if (!FLAGS_enable_semantic_map) {
    ADEBUG << "Not enable semantic map, exit junction_map_evaluator.";
    return false;
  }
  cv::Mat feature_map;
  if (!SemanticMap::Instance()->GetMapById(id, &feature_map)) {
    return false;
  }

  // Build input features for torch
  std::vector<torch::jit::IValue> torch_inputs;
  // Process the feature_map
  cv::cvtColor(feature_map, feature_map, CV_BGR2RGB);
  cv::Mat img_float;
  feature_map.convertTo(img_float, CV_32F, 1.0 / 255);
  torch::Tensor img_tensor = torch::from_blob(img_float.data, {1, 224, 224, 3});
  img_tensor = img_tensor.permute({0, 3, 1, 2});
  img_tensor[0][0] = img_tensor[0][0].sub(0.485).div(0.229);
  img_tensor[0][1] = img_tensor[0][1].sub(0.456).div(0.224);
  img_tensor[0][2] = img_tensor[0][2].sub(0.406).div(0.225);
  // Process junction_exit_mask
  torch::Tensor junction_exit_mask =
      torch::zeros({1, static_cast<int>(feature_values.size())});
  for (size_t i = 0; i < feature_values.size(); ++i) {
    junction_exit_mask[0][i] = static_cast<float>(feature_values[i]);
  }
  torch_inputs.push_back(torch::jit::Tuple::create(
      {img_tensor.to(device_), junction_exit_mask.to(device_)}));

  // Compute probability
  std::vector<double> probability;
  CHECK_NOTNULL(torch_model_ptr_);
  at::Tensor torch_output_tensor =
      torch_model_ptr_->forward(torch_inputs).toTensor().to(torch::kCPU);
  auto torch_output = torch_output_tensor.accessor<float, 2>();
  for (int i = 0; i < torch_output.size(1); ++i) {
    probability.push_back(static_cast<double>(torch_output[0][i]));
  }

  std::unordered_map<std::string, double> junction_exit_prob;
  for (const JunctionExit& junction_exit :
       latest_feature_ptr->junction_feature().junction_exit()) {
    double x =
        junction_exit.exit_position().x() - latest_feature_ptr->position().x();
    double y =
        junction_exit.exit_position().y() - latest_feature_ptr->position().y();
    double angle =
        std::atan2(y, x) - std::atan2(latest_feature_ptr->raw_velocity().y(),
                                      latest_feature_ptr->raw_velocity().x());
    double d_idx = (angle / (2.0 * M_PI) + 1.0 / 24.0) * 12.0;
    int idx = static_cast<int>(floor(d_idx >= 0 ? d_idx : d_idx + 12));
    junction_exit_prob[junction_exit.exit_lane_id()] = probability[idx];
  }

  // assign all lane_sequence probability
  LaneGraph* lane_graph_ptr =
      latest_feature_ptr->mutable_lane()->mutable_lane_graph();
  CHECK_NOTNULL(lane_graph_ptr);
  if (lane_graph_ptr->lane_sequence_size() == 0) {
    AERROR << "Obstacle [" << id << "] has no lane sequences.";
    return false;
  }
  for (int i = 0; i < lane_graph_ptr->lane_sequence_size(); ++i) {
    LaneSequence* lane_sequence_ptr = lane_graph_ptr->mutable_lane_sequence(i);
    CHECK_NOTNULL(lane_sequence_ptr);
    for (const LaneSegment& lane_segment : lane_sequence_ptr->lane_segment()) {
      if (junction_exit_prob.find(lane_segment.lane_id()) !=
          junction_exit_prob.end()) {
        lane_sequence_ptr->set_probability(
            junction_exit_prob[lane_segment.lane_id()]);
      }
    }
  }
  return true;
}

bool JunctionMapEvaluator::ExtractFeatureValues(
    Obstacle* obstacle_ptr, std::vector<double>* feature_values) {
  feature_values->clear();
  feature_values->resize(JUNCTION_FEATURE_SIZE, 0.0);
  Feature* feature_ptr = obstacle_ptr->mutable_latest_feature();
  if (!feature_ptr->has_position()) {
    ADEBUG << "Obstacle [" << obstacle_ptr->id() << "] has no position.";
    return false;
  }
  double heading = std::atan2(feature_ptr->raw_velocity().y(),
                              feature_ptr->raw_velocity().x());
  if (!feature_ptr->has_junction_feature()) {
    AERROR << "Obstacle [" << obstacle_ptr->id()
           << "] has no junction_feature.";
    return false;
  }

  int num_junction_exit = feature_ptr->junction_feature().junction_exit_size();
  for (int i = 0; i < num_junction_exit; ++i) {
    const JunctionExit& junction_exit =
        feature_ptr->junction_feature().junction_exit(i);
    double x = junction_exit.exit_position().x() - feature_ptr->position().x();
    double y = junction_exit.exit_position().y() - feature_ptr->position().y();
    double diff_x = std::cos(-heading) * x - std::sin(-heading) * y;
    double diff_y = std::sin(-heading) * x + std::cos(-heading) * y;
    double angle = std::atan2(diff_y, diff_x);
    double d_idx = (angle / (2.0 * M_PI) + 1.0 / 24.0) * 12.0;
    int idx = static_cast<int>(floor(d_idx >= 0 ? d_idx : d_idx + 12));
    feature_values->at(idx) = 1.0;
  }
  return true;
}

void JunctionMapEvaluator::LoadModel() {
  // TODO(all) uncomment the following when cuda issue is resolved
  // if (torch::cuda::is_available()) {
  //   ADEBUG << "CUDA is available for JunctionMapEvaluator!";
  //   device_ = torch::Device(torch::kCUDA);
  // }
  torch::set_num_threads(1);
  torch_model_ptr_ =
      torch::jit::load(FLAGS_torch_vehicle_junction_map_file, device_);
}

}  // namespace prediction
}  // namespace apollo
