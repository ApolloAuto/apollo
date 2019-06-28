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

#include "modules/prediction/evaluator/vehicle/lane_scanning_evaluator.h"

#include <omp.h>

#include <algorithm>
#include <limits>
#include <utility>

#include "cyber/common/file.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/prediction/common/feature_output.h"
#include "modules/prediction/common/prediction_constants.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_system_gflags.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"

namespace apollo {
namespace prediction {

using apollo::common::TrajectoryPoint;
using apollo::common::adapter::AdapterConfig;
using apollo::common::math::Vec2d;
using apollo::cyber::common::GetProtoFromFile;

LaneScanningEvaluator::LaneScanningEvaluator() : device_(torch::kCPU) {
  evaluator_type_ = ObstacleConf::LANE_SCANNING_EVALUATOR;
  LoadModel();
}

bool LaneScanningEvaluator::Evaluate(Obstacle* obstacle_ptr) {
  std::vector<Obstacle*> dummy_dynamic_env;
  Evaluate(obstacle_ptr, dummy_dynamic_env);
  return true;
}

bool LaneScanningEvaluator::Evaluate(Obstacle* obstacle_ptr,
                                     std::vector<Obstacle*> dynamic_env) {
  // Sanity checks.
  omp_set_num_threads(1);
  CHECK_NOTNULL(obstacle_ptr);

  obstacle_ptr->SetEvaluatorType(evaluator_type_);

  int id = obstacle_ptr->id();
  if (!obstacle_ptr->latest_feature().IsInitialized()) {
    AERROR << "Obstacle [" << id << "] has no latest feature.";
    return false;
  }
  Feature* latest_feature_ptr = obstacle_ptr->mutable_latest_feature();
  CHECK_NOTNULL(latest_feature_ptr);
  if (!latest_feature_ptr->has_lane() ||
      !latest_feature_ptr->lane().has_lane_graph_ordered()) {
    AERROR << "Obstacle [" << id << "] has no lane graph.";
    return false;
  }
  LaneGraph* lane_graph_ptr =
      latest_feature_ptr->mutable_lane()->mutable_lane_graph_ordered();
  CHECK_NOTNULL(lane_graph_ptr);
  if (lane_graph_ptr->lane_sequence_size() == 0) {
    AERROR << "Obstacle [" << id << "] has no lane sequences.";
    return false;
  }
  ADEBUG << "There are " << lane_graph_ptr->lane_sequence_size()
         << " lane sequences to scan.";

  // Extract features, and:
  //  - if in offline mode, save it locally for training.
  //  - if in online mode, pass it through trained model to evaluate.
  std::vector<double> feature_values;
  ExtractFeatures(obstacle_ptr, lane_graph_ptr, &feature_values);
  std::vector<std::string> string_feature_values;
  ExtractStringFeatures(*lane_graph_ptr, &string_feature_values);

  std::vector<double> labels = {0.0};
  if (FLAGS_prediction_offline_mode ==
      PredictionConstants::kDumpDataForLearning) {
    FeatureOutput::InsertDataForLearning(*latest_feature_ptr, feature_values,
                                         string_feature_values, "cruise",
                                         nullptr);
    ADEBUG << "Save extracted features for learning locally.";
    return true;
  }

  feature_values.push_back(static_cast<double>(MAX_NUM_LANE));

  std::vector<torch::jit::IValue> torch_inputs;
  torch::Tensor torch_input =
      torch::zeros({1, static_cast<int>(feature_values.size())});
  for (size_t i = 0; i < feature_values.size(); ++i) {
    torch_input[0][i] = static_cast<float>(feature_values[i]);
  }
  torch_inputs.push_back(std::move(torch_input));
  ModelInference(torch_inputs, torch_lane_scanning_model_ptr_,
                 latest_feature_ptr);
  return true;
}

bool LaneScanningEvaluator::ExtractStringFeatures(
    const LaneGraph& lane_graph,
    std::vector<std::string>* const string_feature_values) {
  for (const LaneSequence& lane_sequence : lane_graph.lane_sequence()) {
    string_feature_values->push_back("|");
    for (int i = lane_sequence.adc_lane_segment_idx();
         i < static_cast<int>(lane_sequence.lane_segment_size()); ++i) {
      string_feature_values->push_back(lane_sequence.lane_segment(i).lane_id());
    }
  }
  return true;
}

bool LaneScanningEvaluator::ExtractFeatures(
    const Obstacle* obstacle_ptr, const LaneGraph* lane_graph_ptr,
    std::vector<double>* feature_values) {
  // Sanity checks.
  CHECK_NOTNULL(obstacle_ptr);
  int id = obstacle_ptr->id();
  CHECK_NOTNULL(lane_graph_ptr);

  // Extract obstacle related features.
  std::vector<double> obstacle_feature_values;
  if (!ExtractObstacleFeatures(obstacle_ptr, &obstacle_feature_values)) {
    ADEBUG << "Failed to extract obstacle features for obs_id = " << id;
  }
  if (obstacle_feature_values.size() != OBSTACLE_FEATURE_SIZE) {
    ADEBUG << "Obstacle [" << id << "] has fewer than "
           << "expected obstacle feature_values "
           << obstacle_feature_values.size() << ".";
    return false;
  }
  ADEBUG << "Obstacle feature size = " << obstacle_feature_values.size();
  feature_values->insert(feature_values->end(), obstacle_feature_values.begin(),
                         obstacle_feature_values.end());

  // Extract static environmental (lane-related) features.
  std::vector<double> static_feature_values;
  std::vector<int> lane_sequence_idx_to_remove;
  if (!ExtractStaticEnvFeatures(obstacle_ptr, lane_graph_ptr,
                                &static_feature_values,
                                &lane_sequence_idx_to_remove)) {
    AERROR << "Failed to extract static environmental features around obs_id = "
           << id;
  }
  if (static_feature_values.size() %
          (SINGLE_LANE_FEATURE_SIZE *
           (LANE_POINTS_SIZE + BACKWARD_LANE_POINTS_SIZE)) !=
      0) {
    AERROR << "Obstacle [" << id << "] has incorrect static env feature size: "
           << static_feature_values.size() << ".";
    return false;
  }
  feature_values->insert(feature_values->end(), static_feature_values.begin(),
                         static_feature_values.end());

  return true;
}

bool LaneScanningEvaluator::ExtractObstacleFeatures(
    const Obstacle* obstacle_ptr, std::vector<double>* feature_values) {
  // Sanity checks.
  CHECK_NOTNULL(obstacle_ptr);
  feature_values->clear();
  FLAGS_cruise_historical_frame_length = 20;
  std::vector<double> has_history(FLAGS_cruise_historical_frame_length, 1.0);
  std::vector<std::pair<double, double>> pos_history(
      FLAGS_cruise_historical_frame_length, std::make_pair(0.0, 0.0));
  std::vector<std::pair<double, double>> vel_history(
      FLAGS_cruise_historical_frame_length, std::make_pair(0.0, 0.0));
  std::vector<std::pair<double, double>> acc_history(
      FLAGS_cruise_historical_frame_length, std::make_pair(0.0, 0.0));
  std::vector<double> vel_heading_history(FLAGS_cruise_historical_frame_length,
                                          0.0);
  std::vector<double> vel_heading_changing_rate_history(
      FLAGS_cruise_historical_frame_length, 0.0);

  // Get obstacle's current position to set up the relative coord. system.
  const Feature& obs_curr_feature = obstacle_ptr->latest_feature();
  double obs_curr_heading = obs_curr_feature.velocity_heading();
  std::pair<double, double> obs_curr_pos = std::make_pair(
      obs_curr_feature.position().x(), obs_curr_feature.position().y());
  double prev_timestamp = obs_curr_feature.timestamp();

  // Starting from the most recent timestamp and going backward.
  ADEBUG << "Obstacle has " << obstacle_ptr->history_size()
         << " history timestamps.";
  for (std::size_t i = 0; i < std::min(obstacle_ptr->history_size(),
                                       FLAGS_cruise_historical_frame_length);
       ++i) {
    const Feature& feature = obstacle_ptr->feature(i);
    if (!feature.IsInitialized()) {
      has_history[i] = 0.0;
      continue;
    }
    if (i != 0 && has_history[i - 1] == 0.0) {
      has_history[i] = 0.0;
      continue;
    }
    // Extract normalized position info.
    if (feature.has_position()) {
      pos_history[i] = WorldCoordToObjCoord(
          std::make_pair(feature.position().x(), feature.position().y()),
          obs_curr_pos, obs_curr_heading);
    } else {
      has_history[i] = 0.0;
    }
    // Extract normalized velocity info.
    if (feature.has_velocity()) {
      auto vel_end = WorldCoordToObjCoord(
          std::make_pair(feature.velocity().x(), feature.velocity().y()),
          obs_curr_pos, obs_curr_heading);
      auto vel_begin = WorldCoordToObjCoord(std::make_pair(0.0, 0.0),
                                            obs_curr_pos, obs_curr_heading);
      vel_history[i] = std::make_pair(vel_end.first - vel_begin.first,
                                      vel_end.second - vel_begin.second);
    } else {
      has_history[i] = 0.0;
    }
    // Extract normalized acceleration info.
    if (feature.has_acceleration()) {
      auto acc_end =
          WorldCoordToObjCoord(std::make_pair(feature.acceleration().x(),
                                              feature.acceleration().y()),
                               obs_curr_pos, obs_curr_heading);
      auto acc_begin = WorldCoordToObjCoord(std::make_pair(0.0, 0.0),
                                            obs_curr_pos, obs_curr_heading);
      acc_history[i] = std::make_pair(acc_end.first - acc_begin.first,
                                      acc_end.second - acc_begin.second);
    } else {
      has_history[i] = 0.0;
    }
    // Extract velocity heading info.
    if (feature.has_velocity_heading()) {
      vel_heading_history[i] =
          WorldAngleToObjAngle(feature.velocity_heading(), obs_curr_heading);
      if (i != 0) {
        vel_heading_changing_rate_history[i] =
            (vel_heading_history[i] - vel_heading_history[i - 1]) /
            (feature.timestamp() - prev_timestamp + FLAGS_double_precision);
        prev_timestamp = feature.timestamp();
      }
    } else {
      has_history[i] = 0.0;
    }
  }

  for (std::size_t i = obstacle_ptr->history_size();
       i < FLAGS_cruise_historical_frame_length; ++i) {
    has_history[i] = 0.0;
  }

  // Update the extracted features into the feature_values vector.
  for (std::size_t i = 0; i < FLAGS_cruise_historical_frame_length; i++) {
    feature_values->push_back(has_history[i]);
    feature_values->push_back(pos_history[i].first);
    feature_values->push_back(pos_history[i].second);
    feature_values->push_back(vel_history[i].first);
    feature_values->push_back(vel_history[i].second);
    feature_values->push_back(acc_history[i].first);
    feature_values->push_back(acc_history[i].second);
    feature_values->push_back(vel_heading_history[i]);
    feature_values->push_back(vel_heading_changing_rate_history[i]);
  }

  return true;
}

bool LaneScanningEvaluator::ExtractStaticEnvFeatures(
    const Obstacle* obstacle_ptr, const LaneGraph* lane_graph_ptr,
    std::vector<double>* feature_values,
    std::vector<int>* lane_sequence_idx_to_remove) {
  // Sanity checks.
  CHECK_NOTNULL(lane_graph_ptr);
  feature_values->clear();

  // Get obstacle's current position to set up the relative coord. system.
  const Feature& obs_curr_feature = obstacle_ptr->latest_feature();
  double obs_curr_heading = obs_curr_feature.velocity_heading();
  std::pair<double, double> obs_curr_pos = std::make_pair(
      obs_curr_feature.position().x(), obs_curr_feature.position().y());

  // Go through every lane-sequence (ordered from left to right) and
  // extract needed features.
  for (int i = 0; i < lane_graph_ptr->lane_sequence_size(); ++i) {
    // Get all the properties of the current lane-sequence.
    // Go through all the lane-points to fill up the feature_values.
    const LaneSequence& lane_sequence = lane_graph_ptr->lane_sequence(i);

    // Extract features from backward lane-points.
    size_t count = 0;
    std::vector<double> backward_feature_values;
    for (int j = lane_sequence.adc_lane_segment_idx(); j >= 0; --j) {
      if (count >= SINGLE_LANE_FEATURE_SIZE * BACKWARD_LANE_POINTS_SIZE) {
        break;
      }
      const LaneSegment& lane_segment = lane_sequence.lane_segment(j);
      int k_starting_idx = lane_segment.lane_point_size() - 1;
      if (j == lane_sequence.adc_lane_segment_idx()) {
        k_starting_idx = std::min(lane_segment.adc_lane_point_idx(),
                                  lane_segment.lane_point_size() - 1);
      }
      for (int k = k_starting_idx; k >= 0; --k) {
        if (count >= SINGLE_LANE_FEATURE_SIZE * BACKWARD_LANE_POINTS_SIZE) {
          break;
        }
        const LanePoint& lane_point = lane_segment.lane_point(k);
        std::pair<double, double> relative_s_l =
            WorldCoordToObjCoord(std::make_pair(lane_point.position().x(),
                                                lane_point.position().y()),
                                 obs_curr_pos, obs_curr_heading);
        double relative_ang =
            WorldAngleToObjAngle(lane_point.heading(), obs_curr_heading);

        backward_feature_values.push_back(lane_point.kappa());
        backward_feature_values.push_back(relative_ang);
        backward_feature_values.push_back(relative_s_l.first);
        backward_feature_values.push_back(relative_s_l.second);

        count += 4;
      }
    }
    // If lane-points are not enough, then extrapolate linearly.
    while (count >= SINGLE_LANE_FEATURE_SIZE * 2 &&
           count < SINGLE_LANE_FEATURE_SIZE * BACKWARD_LANE_POINTS_SIZE) {
      std::size_t s = backward_feature_values.size();
      double relative_l_new =
          2 * backward_feature_values[s - 1] - backward_feature_values[s - 5];
      double relative_s_new =
          2 * backward_feature_values[s - 2] - backward_feature_values[s - 6];
      double relative_ang_new = backward_feature_values[s - 3];

      backward_feature_values.push_back(0.0);
      backward_feature_values.push_back(relative_ang_new);
      backward_feature_values.push_back(relative_s_new);
      backward_feature_values.push_back(relative_l_new);

      count += 4;
    }

    for (int j = static_cast<int>(backward_feature_values.size()) - 1; j >= 0;
         --j) {
      feature_values->push_back(backward_feature_values[j]);
    }

    // Extract features from forward lane-points.
    count = 0;
    for (int j = lane_sequence.adc_lane_segment_idx();
         j < lane_sequence.lane_segment_size(); ++j) {
      if (count >= SINGLE_LANE_FEATURE_SIZE * LANE_POINTS_SIZE) {
        break;
      }
      const LaneSegment& lane_segment = lane_sequence.lane_segment(j);
      int k_starting_idx = 0;
      if (j == lane_sequence.adc_lane_segment_idx()) {
        k_starting_idx = std::min(lane_segment.adc_lane_point_idx(),
                                  lane_segment.lane_point_size() - 1);
      }
      for (int k = k_starting_idx; k < lane_segment.lane_point_size(); ++k) {
        if (count >= SINGLE_LANE_FEATURE_SIZE * LANE_POINTS_SIZE) {
          break;
        }
        const LanePoint& lane_point = lane_segment.lane_point(k);
        std::pair<double, double> relative_s_l =
            WorldCoordToObjCoord(std::make_pair(lane_point.position().x(),
                                                lane_point.position().y()),
                                 obs_curr_pos, obs_curr_heading);
        double relative_ang =
            WorldAngleToObjAngle(lane_point.heading(), obs_curr_heading);

        feature_values->push_back(relative_s_l.second);
        feature_values->push_back(relative_s_l.first);
        feature_values->push_back(relative_ang);
        feature_values->push_back(lane_point.kappa());
        count += 4;
      }
    }
    // If lane-points are not enough, then extrapolate linearly.
    while (count >= SINGLE_LANE_FEATURE_SIZE * 2 &&
           count < SINGLE_LANE_FEATURE_SIZE * LANE_POINTS_SIZE) {
      std::size_t s = feature_values->size();
      double relative_l_new = 2 * feature_values->operator[](s - 4) -
                              feature_values->operator[](s - 8);
      double relative_s_new = 2 * feature_values->operator[](s - 3) -
                              feature_values->operator[](s - 7);
      double relative_ang_new = feature_values->operator[](s - 2);

      feature_values->push_back(relative_l_new);
      feature_values->push_back(relative_s_new);
      feature_values->push_back(relative_ang_new);
      feature_values->push_back(0.0);
      count += 4;
    }
  }

  if (FLAGS_prediction_offline_mode ==
      PredictionConstants::kDumpDataForLearning) {
    // Early exit without appending zero for offline_dataforlearn_dump
    return true;
  }

  size_t max_feature_size =
      LANE_POINTS_SIZE * SINGLE_LANE_FEATURE_SIZE * MAX_NUM_LANE;
  while (feature_values->size() < max_feature_size) {
    feature_values->push_back(0.0);
  }

  return true;
}

void LaneScanningEvaluator::LoadModel() {
  // TODO(all) uncomment the following when cuda issue is resolved
  // if (torch::cuda::is_available()) {
  //   ADEBUG << "CUDA is available";
  //   device_ = torch::Device(torch::kCUDA);
  // }
  torch::set_num_threads(1);
  torch_lane_scanning_model_ptr_ =
      torch::jit::load(FLAGS_torch_vehicle_lane_scanning_file, device_);
}

void LaneScanningEvaluator::ModelInference(
    const std::vector<torch::jit::IValue>& torch_inputs,
    std::shared_ptr<torch::jit::script::Module> torch_model_ptr,
    Feature* feature_ptr) {
  auto torch_output_tensor = torch_model_ptr->forward(torch_inputs).toTensor();
  auto torch_output = torch_output_tensor.accessor<float, 3>();
  for (size_t i = 0; i < SHORT_TERM_TRAJECTORY_SIZE; ++i) {
    TrajectoryPoint point;
    double dx = static_cast<double>(torch_output[0][0][i]);
    double dy =
        static_cast<double>(torch_output[0][0][i + SHORT_TERM_TRAJECTORY_SIZE]);
    Vec2d offset(dx, dy);
    Vec2d rotated_offset = offset.rotate(feature_ptr->velocity_heading());
    double point_x = feature_ptr->position().x() + rotated_offset.x();
    double point_y = feature_ptr->position().y() + rotated_offset.y();
    point.mutable_path_point()->set_x(point_x);
    point.mutable_path_point()->set_y(point_y);
    point.set_relative_time(static_cast<double>(i) *
                            FLAGS_prediction_trajectory_time_resolution);
    feature_ptr->add_short_term_predicted_trajectory_points()->CopyFrom(point);
  }
}

}  // namespace prediction
}  // namespace apollo
