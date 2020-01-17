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

#include <omp.h>

#include <algorithm>
#include <unordered_map>
#include <utility>

#include "cyber/common/file.h"
#include "modules/common/adapters/proto/adapter_config.pb.h"
#include "modules/common/math/vec2d.h"
#include "modules/prediction/common/feature_output.h"
#include "modules/prediction/common/prediction_constants.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_map.h"
#include "modules/prediction/common/prediction_system_gflags.h"
#include "modules/prediction/common/prediction_util.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"

namespace apollo {
namespace prediction {

using apollo::prediction::math_util::ComputePolynomial;
using apollo::prediction::math_util::EvaluateCubicPolynomial;

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

JunctionMLPEvaluator::JunctionMLPEvaluator() : device_(torch::kCPU) {
  evaluator_type_ = ObstacleConf::JUNCTION_MLP_EVALUATOR;
  LoadModel();
}

void JunctionMLPEvaluator::Clear() {}

bool JunctionMLPEvaluator::Evaluate(Obstacle* obstacle_ptr,
                                    ObstaclesContainer* obstacles_container) {
  // Sanity checks.
  omp_set_num_threads(1);
  Clear();
  CHECK_NOTNULL(obstacle_ptr);

  obstacle_ptr->SetEvaluatorType(evaluator_type_);

  int id = obstacle_ptr->id();
  if (!obstacle_ptr->latest_feature().IsInitialized()) {
    AERROR << "Obstacle [" << id << "] has no latest feature.";
    return false;
  }
  Feature* latest_feature_ptr = obstacle_ptr->mutable_latest_feature();
  CHECK_NOTNULL(latest_feature_ptr);

  // Assume obstacle is NOT closed to any junction exit
  if (!latest_feature_ptr->has_junction_feature() ||
      latest_feature_ptr->junction_feature().junction_exit_size() < 1) {
    ADEBUG << "Obstacle [" << id << "] has no junction_exit.";
    return false;
  }

  std::vector<double> feature_values;
  ExtractFeatureValues(obstacle_ptr, obstacles_container, &feature_values);

  // Insert features to DataForLearning
  if (FLAGS_prediction_offline_mode ==
      PredictionConstants::kDumpDataForLearning) {
    FeatureOutput::InsertDataForLearning(*latest_feature_ptr, feature_values,
                                         "junction", nullptr);
    ADEBUG << "Save extracted features for learning locally.";
    return true;  // Skip Compute probability for offline mode
  }
  std::vector<torch::jit::IValue> torch_inputs;
  int input_dim = static_cast<int>(
      OBSTACLE_FEATURE_SIZE + EGO_VEHICLE_FEATURE_SIZE + JUNCTION_FEATURE_SIZE);
  torch::Tensor torch_input = torch::zeros({1, input_dim});
  for (size_t i = 0; i < feature_values.size(); ++i) {
    torch_input[0][i] = static_cast<float>(feature_values[i]);
  }
  torch_inputs.push_back(std::move(torch_input.to(device_)));
  std::vector<double> probability;
  if (latest_feature_ptr->junction_feature().junction_exit_size() > 1) {
    at::Tensor torch_output_tensor =
        torch_model_.forward(torch_inputs).toTensor().to(torch::kCPU);
    auto torch_output = torch_output_tensor.accessor<float, 2>();
    for (int i = 0; i < torch_output.size(1); ++i) {
      probability.push_back(static_cast<double>(torch_output[0][i]));
    }
  } else {
    for (int i = 0; i < 12; ++i) {
      probability.push_back(feature_values[OBSTACLE_FEATURE_SIZE +
                                           EGO_VEHICLE_FEATURE_SIZE + 8 * i]);
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
  if (lane_graph_ptr->lane_sequence().empty()) {
    AERROR << "Obstacle [" << id << "] has no lane sequences.";
    return false;
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

void JunctionMLPEvaluator::ExtractFeatureValues(
    Obstacle* obstacle_ptr, ObstaclesContainer* obstacles_container,
    std::vector<double>* feature_values) {
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

  std::vector<double> ego_vehicle_feature_values;
  SetEgoVehicleFeatureValues(obstacle_ptr, obstacles_container,
                             &ego_vehicle_feature_values);
  if (ego_vehicle_feature_values.size() != EGO_VEHICLE_FEATURE_SIZE) {
    AERROR << "Obstacle [" << id << "] has fewer than "
           << "expected ego vehicle feature_values"
           << ego_vehicle_feature_values.size() << ".";
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

  feature_values->insert(feature_values->end(), obstacle_feature_values.begin(),
                         obstacle_feature_values.end());
  feature_values->insert(feature_values->end(),
                         ego_vehicle_feature_values.begin(),
                         ego_vehicle_feature_values.end());
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
  std::pair<double, double> obs_curr_pos =
      std::make_pair(feature.position().x(), feature.position().y());
  double obs_curr_heading = feature.velocity_heading();
  bool has_history = false;
  std::vector<std::pair<double, double>> pos_history(
      FLAGS_junction_historical_frame_length, std::make_pair(0.0, 0.0));

  if (obstacle_ptr->history_size() > FLAGS_junction_historical_frame_length) {
    has_history = true;
    for (std::size_t i = 0; i < FLAGS_junction_historical_frame_length; ++i) {
      const Feature& feature = obstacle_ptr->feature(i + 1);
      if (!feature.IsInitialized()) {
        has_history = false;
        break;
      }
      if (feature.has_position()) {
        pos_history[i] = WorldCoordToObjCoord(
            std::make_pair(feature.position().x(), feature.position().y()),
            obs_curr_pos, obs_curr_heading);
      }
    }
  }

  feature_values->push_back(feature.speed());
  feature_values->push_back(feature.acc());
  feature_values->push_back(feature.junction_feature().junction_range());
  if (has_history) {
    feature_values->push_back(1.0);
  } else {
    feature_values->push_back(0.0);
  }
  for (std::size_t i = 0; i < FLAGS_junction_historical_frame_length; i++) {
    feature_values->push_back(pos_history[i].first);
    feature_values->push_back(pos_history[i].second);
  }
}

void JunctionMLPEvaluator::SetEgoVehicleFeatureValues(
    Obstacle* obstacle_ptr, ObstaclesContainer* obstacles_container,
    std::vector<double>* const feature_values) {
  feature_values->clear();
  *feature_values = std::vector<double>(4, 0.0);
  auto ego_pose_obstacle_ptr =
      obstacles_container->GetObstacle(FLAGS_ego_vehicle_id);
  if (ego_pose_obstacle_ptr == nullptr ||
      ego_pose_obstacle_ptr->history_size() == 0) {
    (*feature_values)[0] = 100.0;
    (*feature_values)[1] = 100.0;
    return;
  }
  const auto ego_position = ego_pose_obstacle_ptr->latest_feature().position();
  const auto ego_velocity = ego_pose_obstacle_ptr->latest_feature().velocity();
  CHECK_GT(obstacle_ptr->history_size(), 0);
  const Feature& obstacle_feature = obstacle_ptr->latest_feature();
  apollo::common::math::Vec2d ego_relative_position(
      ego_position.x() - obstacle_feature.position().x(),
      ego_position.y() - obstacle_feature.position().y());
  apollo::common::math::Vec2d ego_relative_velocity(ego_velocity.x(),
                                                    ego_velocity.y());
  ego_relative_velocity.rotate(-obstacle_feature.velocity_heading());
  (*feature_values)[0] = ego_relative_position.x();
  (*feature_values)[1] = ego_relative_position.y();
  (*feature_values)[2] = ego_relative_velocity.x();
  (*feature_values)[3] = ego_relative_velocity.y();
  ADEBUG << "ego relative pos = {" << ego_relative_position.x() << ", "
         << ego_relative_position.y() << "} "
         << "ego_relative_velocity = {" << ego_relative_velocity.x() << ", "
         << ego_relative_velocity.y() << "}";
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
    feature_values->push_back(0.0);
    feature_values->push_back(0.0);
    feature_values->push_back(1.0);
    feature_values->push_back(1.0);
    feature_values->push_back(1.0);
    feature_values->push_back(0.0);
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
    double diff_heading =
        apollo::common::math::AngleDiff(heading, junction_exit.exit_heading());
    double angle = std::atan2(diff_y, diff_x);
    double d_idx = (angle / (2.0 * M_PI) + 1.0 / 24.0) * 12.0;
    int idx = static_cast<int>(floor(d_idx >= 0 ? d_idx : d_idx + 12));
    double speed = std::max(0.1, feature_ptr->speed());
    double exit_time = std::hypot(diff_x, diff_y) / speed;
    std::array<double, 2> start_x = {0, speed};
    std::array<double, 2> end_x = {diff_x, std::cos(diff_heading) * speed};
    std::array<double, 2> start_y = {0, 0};
    std::array<double, 2> end_y = {diff_y, std::sin(diff_heading) * speed};
    std::array<double, 4> x_coeffs =
        ComputePolynomial<3>(start_x, end_x, exit_time);
    std::array<double, 4> y_coeffs =
        ComputePolynomial<3>(start_y, end_y, exit_time);
    double t = 0.0;
    double cost = 0.0;
    while (t <= exit_time) {
      double x_1 = EvaluateCubicPolynomial(x_coeffs, t, 1);
      double x_2 = EvaluateCubicPolynomial(x_coeffs, t, 2);
      double y_1 = EvaluateCubicPolynomial(y_coeffs, t, 1);
      double y_2 = EvaluateCubicPolynomial(y_coeffs, t, 2);
      // cost = curvature * v^2
      cost = std::max(cost,
                      std::abs(x_1 * y_2 - y_1 * x_2) / std::hypot(x_1, y_1));
      t += FLAGS_prediction_trajectory_time_resolution;
    }
    feature_values->operator[](idx * 8) = 1.0;
    feature_values->operator[](idx * 8 + 1) = diff_x / junction_range;
    feature_values->operator[](idx * 8 + 2) = diff_y / junction_range;
    feature_values->operator[](idx * 8 + 3) =
        std::sqrt(diff_x * diff_x + diff_y * diff_y) / junction_range;
    feature_values->operator[](idx * 8 + 4) = diff_x;
    feature_values->operator[](idx * 8 + 5) = diff_y;
    feature_values->operator[](idx * 8 + 6) = diff_heading;
    feature_values->operator[](idx * 8 + 7) = cost;
  }
}

void JunctionMLPEvaluator::LoadModel() {
  if (FLAGS_use_cuda && torch::cuda::is_available()) {
    ADEBUG << "CUDA is available";
    device_ = torch::Device(torch::kCUDA);
  }
  torch::set_num_threads(1);
  torch_model_ =
      torch::jit::load(FLAGS_torch_vehicle_junction_mlp_file, device_);
}

}  // namespace prediction
}  // namespace apollo
