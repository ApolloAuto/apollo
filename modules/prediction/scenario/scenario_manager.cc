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

#include "modules/prediction/scenario/scenario_manager.h"

#include <algorithm>

#include "modules/prediction/common/prediction_gflags.h"

namespace apollo {
namespace prediction {

using common::adapter::AdapterConfig;
using common::math::Vec2d;
using common::math::Box2d;
using apollo::perception::PerceptionObstacle;

ScenarioManager::ScenarioManager() {}

void ScenarioManager::Run() {
  auto environment_features = FeatureExtractor::ExtractEnvironmentFeatures();

  auto ptr_scenario_features = ScenarioAnalyzer::Analyze(environment_features);

  current_scenario_ = ptr_scenario_features->scenario();

  if (FLAGS_enable_prioritize_obstacles) {
    PrioritizeObstacles(environment_features, ptr_scenario_features);
  }
  // TODO(all) other functionalities including lane, junction filters
}

const Scenario& ScenarioManager::scenario() const { return current_scenario_; }

void ScenarioManager::PrioritizeObstacles(
    const EnvironmentFeatures& environment_features,
    const std::shared_ptr<ScenarioFeatures> ptr_scenario_features) {
  auto obstacles_container = ContainerManager::Instance()->GetContainer<
      ObstaclesContainer>(AdapterConfig::PERCEPTION_OBSTACLES);

  if (obstacles_container == nullptr) {
    AERROR << "Obstacles container pointer is a null pointer.";
    return;
  }

  auto pose_container = ContainerManager::Instance()->GetContainer<
      PoseContainer>(AdapterConfig::LOCALIZATION);
  if (pose_container == nullptr) {
    AERROR << "Pose container pointer is a null pointer.";
    return;
  }

  const PerceptionObstacle* pose_obstacle_ptr =
      pose_container->ToPerceptionObstacle();
  if (pose_obstacle_ptr == nullptr) {
    AERROR << "Pose obstacle pointer is a null pointer.";
    return;
  }

  double pose_theta = pose_obstacle_ptr->theta();
  double pose_x = pose_obstacle_ptr->position().x();
  double pose_y = pose_obstacle_ptr->position().y();

  ADEBUG << "Get pose (" << pose_x << ", " << pose_y
         << ", " << pose_theta << ")";

  // Build rectangular scan_area
  Box2d scan_box({pose_x + FLAGS_scan_length / 2.0 * std::cos(pose_theta),
                  pose_y + FLAGS_scan_length / 2.0 * std::sin(pose_theta)},
                  pose_theta, FLAGS_scan_length, FLAGS_scan_width);

  const auto& obstacle_ids =
      obstacles_container->curr_frame_predictable_obstacle_ids();

  for (const int& obstacle_id : obstacle_ids) {
    Obstacle* obstacle_ptr = obstacles_container->GetObstacle(obstacle_id);
    if (obstacle_ptr->history_size() == 0) {
      AERROR << "Obstacle [" << obstacle_ptr->id() << "] has no feature.";
      continue;
    }
    Feature* latest_feature_ptr = obstacle_ptr->mutable_latest_feature();
    double obstacle_x = latest_feature_ptr->position().x();
    double obstacle_y = latest_feature_ptr->position().y();
    Vec2d ego_to_obstacle_vec(obstacle_x - pose_x, obstacle_y - pose_y);
    Vec2d ego_vec = Vec2d::CreateUnitVec2d(pose_theta);
    double s = ego_to_obstacle_vec.InnerProd(ego_vec);

    double pedestrian_like_nearby_lane_radius =
        FLAGS_pedestrian_nearby_lane_search_radius;
    bool is_near_lane = PredictionMap::HasNearbyLane(obstacle_x, obstacle_y,
        pedestrian_like_nearby_lane_radius);

    // Decide if we need consider this obstacle
    bool is_in_scan_area = scan_box.IsPointIn({obstacle_x, obstacle_y});
    bool is_on_lane = obstacle_ptr->IsOnLane();
    bool is_pedestrian_like_in_front_near_lanes =
        s > FLAGS_back_dist_ignore_ped &&
        (latest_feature_ptr->type() == PerceptionObstacle::PEDESTRIAN ||
         latest_feature_ptr->type() == PerceptionObstacle::BICYCLE ||
         latest_feature_ptr->type() == PerceptionObstacle::UNKNOWN ||
         latest_feature_ptr->type() == PerceptionObstacle::UNKNOWN_MOVABLE) &&
        is_near_lane;
    bool is_near_junction = obstacle_ptr->IsNearJunction();

    bool need_consider = is_in_scan_area || is_on_lane || is_near_junction ||
                         is_pedestrian_like_in_front_near_lanes;

    if (!need_consider) {
      latest_feature_ptr->mutable_priority()->set_priority(
            ObstaclePriority::IGNORE);
    } else {
      latest_feature_ptr->mutable_priority()->set_priority(
            ObstaclePriority::NORMAL);
    }
  }
}

void ScenarioManager::PrioritizeObstaclesForCruiseScenario(
    const EnvironmentFeatures& environment_features,
    const std::shared_ptr<CruiseScenarioFeatures> cruise_scenario_features,
    ObstaclesContainer* ptr_obstacle_contrainer) {
  const auto& obstacle_ids =
      ptr_obstacle_contrainer->curr_frame_predictable_obstacle_ids();

  for (const int& obstacle_id : obstacle_ids) {
    Obstacle* obstacle_ptr = ptr_obstacle_contrainer->GetObstacle(obstacle_id);
    if (obstacle_ptr->history_size() == 0) {
      AERROR << "Obstacle [" << obstacle_ptr->id() << "] has no feature.";
      continue;
    }
    Feature* latest_feature_ptr = obstacle_ptr->mutable_latest_feature();
    if (obstacle_ptr->IsOnLane()) {
      bool has_lane_of_interest = false;
      for (const auto& curr_lane :
           latest_feature_ptr->lane().current_lane_feature()) {
        const auto& curr_lane_id = curr_lane.lane_id();
        if (cruise_scenario_features->IsLaneOfInterest(curr_lane_id)) {
          has_lane_of_interest = true;
          break;
        }
      }
      if (!has_lane_of_interest) {
        for (const auto& nearby_lane :
             latest_feature_ptr->lane().nearby_lane_feature()) {
          const auto& nearby_lane_id = nearby_lane.lane_id();
          if (cruise_scenario_features->IsLaneOfInterest(nearby_lane_id)) {
            has_lane_of_interest = true;
            break;
          }
        }
      }
      if (!has_lane_of_interest) {
        latest_feature_ptr->mutable_priority()->set_priority(
            ObstaclePriority::IGNORE);
      }
    } else {
      // Obstacle has neither lane nor neighbor lanes
      // Filter obstacle by its relative position to ego vehicle
      double ego_x = environment_features.get_ego_position().x();
      double ego_y = environment_features.get_ego_position().y();
      double ego_heading = environment_features.get_ego_heading();
      double ego_speed = environment_features.get_ego_speed();
      double obstacle_x = latest_feature_ptr->position().x();
      double obstacle_y = latest_feature_ptr->position().y();
      Vec2d ego_to_obstacle_vec(obstacle_x - ego_x, obstacle_y - ego_y);
      Vec2d ego_vec = Vec2d::CreateUnitVec2d(ego_heading);
      double l = ego_vec.CrossProd(ego_to_obstacle_vec);
      double s = ego_to_obstacle_vec.InnerProd(ego_vec);
      if (std::abs(l) > 10.0 || s > std::max(20.0, ego_speed * 5.0) ||
          s < 0.0) {
        latest_feature_ptr->mutable_priority()->set_priority(
            ObstaclePriority::IGNORE);
      }
    }
  }
}

}  // namespace prediction
}  // namespace apollo
