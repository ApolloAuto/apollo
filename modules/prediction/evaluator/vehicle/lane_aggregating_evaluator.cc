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

#include "modules/prediction/evaluator/vehicle/lane_aggregating_evaluator.h"

#include <algorithm>
#include <utility>
#include <vector>

#include "modules/common/math/vec2d.h"
#include "modules/prediction/common/feature_output.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_system_gflags.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"
#include "modules/prediction/container/pose/pose_container.h"

namespace apollo {
namespace prediction {

using apollo::common::adapter::AdapterConfig;
using apollo::common::math::Vec2d;
using apollo::common::Point3D;
using apollo::common::TrajectoryPoint;
using apollo::perception::PerceptionObstacle;
using apollo::perception::PerceptionObstacles;

LaneAggregatingEvaluator::LaneAggregatingEvaluator()
    : device_(torch::kCPU) {
  // LoadModel();
}

bool LaneAggregatingEvaluator::Evaluate(Obstacle* obstacle_ptr) {
  // Sanity checks.
  CHECK_NOTNULL(obstacle_ptr);
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

  // Extract features, and do model inferencing.

  return true;
}

bool LaneAggregatingEvaluator::ExtractObstacleFeatures(
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

}  // namespace prediction
}  // namespace apollo
