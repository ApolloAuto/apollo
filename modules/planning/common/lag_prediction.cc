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

/**
 * @file
 **/

#include "modules/planning/common/lag_prediction.h"

#include <algorithm>

#include "modules/common/adapters/adapter_manager.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::adapter::AdapterManager;
using apollo::perception::PerceptionObstacle;
using apollo::prediction::PredictionObstacle;
using apollo::prediction::PredictionObstacles;

LagPrediction::LagPrediction(uint32_t min_appear_num,
                             uint32_t max_disappear_num)
    : min_appear_num_(min_appear_num), max_disappear_num_(max_disappear_num) {
  if (AdapterManager::GetPredictionConfig().message_history_limit() <
      static_cast<int32_t>(min_appear_num_)) {
    AWARN << "Prediction adapter history limit is "
          << AdapterManager::GetPredictionConfig().message_history_limit()
          << ", but an obstacle need to be observed at least "
          << min_appear_num_ << " times";
    return;
  }
}

void LagPrediction::GetLaggedPrediction(PredictionObstacles* obstacles) const {
  obstacles->mutable_prediction_obstacle()->Clear();
  if (!AdapterManager::GetPrediction() ||
      AdapterManager::GetPrediction()->Empty()) {
    return;
  }
  const auto& prediction = *(AdapterManager::GetPrediction());
  if (!AdapterManager::GetLocalization() ||
      AdapterManager::GetLocalization()->Empty()) {  // no localization
    obstacles->CopyFrom(prediction.GetLatestObserved());
    return;
  }
  const auto adc_position =
      AdapterManager::GetLocalization()->GetLatestObserved().pose().position();
  const auto latest_prediction = (*prediction.begin());
  const double timestamp = latest_prediction->header().timestamp_sec();

  std::unordered_set<int> protected_obstacles;
  for (const auto& obstacle : latest_prediction->prediction_obstacle()) {
    const auto& perception = obstacle.perception_obstacle();
    if (perception.confidence() < FLAGS_perception_confidence_threshold &&
        perception.type() != PerceptionObstacle::VEHICLE) {
      continue;
    }
    double distance =
        common::util::DistanceXY(perception.position(), adc_position);
    if (distance < FLAGS_lag_prediction_protection_distance) {
      protected_obstacles.insert(obstacle.perception_obstacle().id());
      // add protected obstacle
      AddObstacleToPrediction(0.0, obstacle, obstacles);
    }
  }

  std::unordered_map<int, LagInfo> obstacle_lag_info;
  int index = 0;  // data in begin() is the most recent data
  for (auto it = prediction.begin(); it != prediction.end(); ++it, ++index) {
    for (const auto& obstacle : (*it)->prediction_obstacle()) {
      const auto& perception = obstacle.perception_obstacle();
      auto id = perception.id();
      if (perception.confidence() < FLAGS_perception_confidence_threshold &&
          perception.type() != PerceptionObstacle::VEHICLE) {
        continue;
      }
      if (protected_obstacles.count(id) > 0) {
        continue;  // don't need to count the already added protected obstacle
      }
      auto& info = obstacle_lag_info[id];
      ++info.count;
      if ((*it)->header().timestamp_sec() > info.last_observed_time) {
        info.last_observed_time = (*it)->header().timestamp_sec();
        info.last_observed_seq = index;
        info.obstacle_ptr = &obstacle;
      }
    }
  }

  obstacles->mutable_header()->CopyFrom(latest_prediction->header());
  obstacles->mutable_header()->set_module_name("lag_prediction");
  obstacles->set_perception_error_code(
      latest_prediction->perception_error_code());
  obstacles->set_start_timestamp(latest_prediction->start_timestamp());
  obstacles->set_end_timestamp(latest_prediction->end_timestamp());
  bool apply_lag = std::distance(prediction.begin(), prediction.end()) >=
                   static_cast<int32_t>(min_appear_num_);
  for (const auto& iter : obstacle_lag_info) {
    if (apply_lag && iter.second.count < min_appear_num_) {
      continue;
    }
    if (apply_lag && iter.second.last_observed_seq > max_disappear_num_) {
      continue;
    }
    AddObstacleToPrediction(timestamp - iter.second.last_observed_time,
                            *(iter.second.obstacle_ptr), obstacles);
  }
}

void LagPrediction::AddObstacleToPrediction(
    double delay_sec, const prediction::PredictionObstacle& history_obstacle,
    prediction::PredictionObstacles* obstacles) const {
  auto* obstacle = obstacles->add_prediction_obstacle();
  if (delay_sec <= 1e-6) {
    obstacle->CopyFrom(history_obstacle);
    return;
  }
  obstacle->mutable_perception_obstacle()->CopyFrom(
      history_obstacle.perception_obstacle());
  for (const auto& hist_trajectory : history_obstacle.trajectory()) {
    auto* traj = obstacle->add_trajectory();
    for (const auto& hist_point : hist_trajectory.trajectory_point()) {
      if (hist_point.relative_time() < delay_sec) {
        continue;
      }
      auto* point = traj->add_trajectory_point();
      point->CopyFrom(hist_point);
      point->set_relative_time(hist_point.relative_time() - delay_sec);
    }
    if (traj->trajectory_point_size() <= 0) {
      obstacle->mutable_trajectory()->RemoveLast();
      continue;
    }
    traj->set_probability(hist_trajectory.probability());
  }
  if (obstacle->trajectory_size() <= 0) {
    obstacles->mutable_prediction_obstacle()->RemoveLast();
    return;
  }
  obstacle->set_timestamp(history_obstacle.timestamp());
  obstacle->set_predicted_period(history_obstacle.predicted_period());
}

}  // namespace planning
}  // namespace apollo
