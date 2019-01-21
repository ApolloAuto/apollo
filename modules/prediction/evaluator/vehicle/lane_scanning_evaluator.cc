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

#include <algorithm>
#include <limits>
#include <utility>

#include "modules/common/util/file.h"
#include "modules/prediction/common/feature_output.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_system_gflags.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"
#include "modules/prediction/evaluator/vehicle/lane_scanning_evaluator.h"

namespace apollo {
namespace prediction {

using apollo::common::adapter::AdapterConfig;
using apollo::common::util::GetProtoFromFile;

LaneScanningEvaluator::LaneScanningEvaluator() {
}

void LaneScanningEvaluator::Evaluate(Obstacle* obstacle_ptr) {
  std::vector<Obstacle*> dummy_dynamic_env;
  Evaluate(obstacle_ptr, dummy_dynamic_env);
}

void LaneScanningEvaluator::Evaluate(
    Obstacle* obstacle_ptr, std::vector<Obstacle*> dynamic_env) {
  // Sanity checks.
  CHECK_NOTNULL(obstacle_ptr);
  int id = obstacle_ptr->id();
  if (!obstacle_ptr->latest_feature().IsInitialized()) {
    AERROR << "Obstacle [" << id << "] has no latest feature.";
    return;
  }
  Feature* latest_feature_ptr = obstacle_ptr->mutable_latest_feature();
  CHECK_NOTNULL(latest_feature_ptr);
  if (!latest_feature_ptr->has_lane() ||
      !latest_feature_ptr->lane().has_lane_graph_ordered()) {
    AERROR << "Obstacle [" << id << "] has no lane graph.";
    return;
  }
  LaneGraph* lane_graph_ptr =
      latest_feature_ptr->mutable_lane()->mutable_lane_graph_ordered();
  CHECK_NOTNULL(lane_graph_ptr);
  if (lane_graph_ptr->lane_sequence_size() == 0) {
    AERROR << "Obstacle [" << id << "] has no lane sequences.";
    return;
  }
  ADEBUG << "There are " << lane_graph_ptr->lane_sequence_size()
         << " lane sequences to scan.";

  // Extract features, and:
  //  - if in offline mode, save it locally for training.
  //  - if in online mode, pass it through trained model to evaluate.
  std::vector<double> feature_values;
  ExtractFeatures(obstacle_ptr, lane_graph_ptr, &feature_values);
  std::vector<double> labels = {0.0};
  if (FLAGS_prediction_offline_dataforlearning) {
    FeatureOutput::InsertDataForLearning(*latest_feature_ptr, feature_values,
                                         labels);
    ADEBUG << "Save extracted features for learning locally.";
  } else {
    // TODO(jiacheng): once the model is trained, implement this online part.
  }
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
  feature_values->insert(feature_values->end(),
                         obstacle_feature_values.begin(),
                         obstacle_feature_values.end());

  // Extract static environmental (lane-related) features.
  std::vector<double> static_feature_values;
  if (!ExtractStaticEnvFeatures(obstacle_ptr, lane_graph_ptr,
                                &static_feature_values)) {
    ADEBUG << "Failed to extract static environmental features around obs_id = "
           << id;
  }
  if (static_feature_values.size() %
      (SINGLE_LANE_FEATURE_SIZE * LANE_POINTS_SIZE) != 0) {
    ADEBUG << "Obstacle [" << id << "] has incorrect static env feature size: "
           << static_feature_values.size() << ".";
    return false;
  }
  feature_values->insert(feature_values->end(),
                         static_feature_values.begin(),
                         static_feature_values.end());

  return true;
}

bool LaneScanningEvaluator::ExtractObstacleFeatures(
    const Obstacle* obstacle_ptr, std::vector<double>* feature_values) {
  // Sanity checks.
  CHECK_NOTNULL(obstacle_ptr);
  feature_values->clear();
  std::vector<double> has_history(FLAGS_cruise_historical_frame_length, 1.0);
  std::vector<std::pair<double, double>> pos_history
      (FLAGS_cruise_historical_frame_length, std::make_pair(0.0, 0.0));
  std::vector<std::pair<double, double>> vel_history
      (FLAGS_cruise_historical_frame_length, std::make_pair(0.0, 0.0));
  std::vector<std::pair<double, double>> acc_history
      (FLAGS_cruise_historical_frame_length, std::make_pair(0.0, 0.0));
  std::vector<double> vel_heading_history
      (FLAGS_cruise_historical_frame_length, 0.0);
  std::vector<double> vel_heading_changing_rate_history
      (FLAGS_cruise_historical_frame_length, 0.0);

  // Get obstacle's current position to set up the relative coord. system.
  const Feature& obs_curr_feature = obstacle_ptr->latest_feature();
  double obs_curr_heading = obs_curr_feature.velocity_heading();
  std::pair<double, double> obs_curr_pos =
      std::make_pair(obs_curr_feature.position().x(),
                     obs_curr_feature.position().y());
  double prev_timestamp = obs_curr_feature.timestamp();

  // Starting from the most recent timestamp and going backward.
  ADEBUG << "Obstacle has " << obstacle_ptr->history_size()
         << " history timestamps.";
  for (std::size_t i = 0; i < std::min(obstacle_ptr->history_size(),
       FLAGS_cruise_historical_frame_length); ++i) {
    const Feature& feature = obstacle_ptr->feature(i);
    if (!feature.IsInitialized()) {
      has_history[i] = 0.0;
      continue;
    }
    if (i != 0 && has_history[i-1] == 0.0) {
      has_history[i] = 0.0;
      continue;
    }
    // Extract normalized position info.
    if (feature.has_position()) {
      pos_history[i] = WorldCoordToObjCoord
          (std::make_pair(feature.position().x(), feature.position().y()),
           obs_curr_pos, obs_curr_heading);
    } else {
      has_history[i] = 0.0;
    }
    // Extract normalized velocity info.
    if (feature.has_velocity()) {
      auto vel_end = WorldCoordToObjCoord
          (std::make_pair(feature.velocity().x(),
                          feature.velocity().y()),
           obs_curr_pos, obs_curr_heading);
      auto vel_begin = WorldCoordToObjCoord
          (std::make_pair(0.0, 0.0), obs_curr_pos, obs_curr_heading);
      vel_history[i] = std::make_pair(vel_end.first - vel_begin.first,
                                      vel_end.second - vel_begin.second);
    } else {
      has_history[i] = 0.0;
    }
    // Extract normalized acceleration info.
    if (feature.has_acceleration()) {
      auto acc_end = WorldCoordToObjCoord
          (std::make_pair(feature.acceleration().x(),
                          feature.acceleration().y()),
           obs_curr_pos, obs_curr_heading);
      auto acc_begin = WorldCoordToObjCoord
          (std::make_pair(0.0, 0.0), obs_curr_pos, obs_curr_heading);
      acc_history[i] = std::make_pair(acc_end.first - acc_begin.first,
                                      acc_end.second - acc_begin.second);
    } else {
      has_history[i] = 0.0;
    }
    // Extract velocity heading info.
    if (feature.has_velocity_heading()) {
      vel_heading_history[i] = WorldAngleToObjAngle
          (feature.velocity_heading(), obs_curr_heading);
      if (i != 0) {
        vel_heading_changing_rate_history[i] =
            (vel_heading_history[i] - vel_heading_history[i-1]) /
            (feature.timestamp() - prev_timestamp + FLAGS_double_precision);
        prev_timestamp = feature.timestamp();
      }
    } else {
      has_history[i] = 0.0;
    }
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
    std::vector<double>* feature_values) {
  // Sanity checks.
  CHECK_NOTNULL(lane_graph_ptr);
  feature_values->clear();

  // Get obstacle's current position to set up the relative coord. system.
  const Feature& obs_curr_feature = obstacle_ptr->latest_feature();
  double obs_curr_heading = obs_curr_feature.velocity_heading();
  std::pair<double, double> obs_curr_pos =
      std::make_pair(obs_curr_feature.position().x(),
                     obs_curr_feature.position().y());

  // Go through every lane-sequence (ordered from left to right) and
  // extract needed features.
  for (int i = 0; i < lane_graph_ptr->lane_sequence_size(); ++i) {
    size_t count = 0;
    // Go through all the lane-points to fill up the feature_values.
    const LaneSequence& lane_sequence = lane_graph_ptr->lane_sequence(i);
    for (int j = 0; j < lane_sequence.lane_segment_size(); ++j) {
      if (count >= SINGLE_LANE_FEATURE_SIZE * LANE_POINTS_SIZE) {
        break;
      }
      const LaneSegment& lane_segment = lane_sequence.lane_segment(j);
      for (int k = 0; k < lane_segment.lane_point_size(); ++k) {
        if (count >= SINGLE_LANE_FEATURE_SIZE * LANE_POINTS_SIZE) {
          break;
        }
        const LanePoint& lane_point = lane_segment.lane_point(k);
        std::pair<double, double> relative_s_l = WorldCoordToObjCoord(
            std::make_pair(lane_point.position().x(),
                           lane_point.position().y()),
            obs_curr_pos, obs_curr_heading);
        double relative_ang = WorldAngleToObjAngle(lane_point.heading(),
                                                   obs_curr_heading);

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
      double relative_l_new = 2 * feature_values->operator[](s - 5) -
                              feature_values->operator[](s - 10);
      double relative_s_new = 2 * feature_values->operator[](s - 4) -
                              feature_values->operator[](s - 9);
      double relative_ang_new = feature_values->operator[](s - 3);

      feature_values->push_back(relative_l_new);
      feature_values->push_back(relative_s_new);
      feature_values->push_back(relative_ang_new);
      feature_values->push_back(0.0);
      count += 4;
    }
  }

  return true;
}

}  // namespace prediction
}  // namespace apollo
