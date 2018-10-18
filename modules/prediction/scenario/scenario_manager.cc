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
#include <cmath>

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/math/vec2d.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/container/container_manager.h"

namespace apollo {
namespace prediction {

using common::adapter::AdapterConfig;
using common::math::Vec2d;

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
  ObstaclesContainer* obstacles_container = dynamic_cast<ObstaclesContainer*>(
      ContainerManager::Instance()->GetContainer(
          AdapterConfig::PERCEPTION_OBSTACLES));

  if (obstacles_container == nullptr) {
    AERROR << "Obstacles container pointer is a null pointer.";
    return;
  }
  const auto& scenario_type = ptr_scenario_features->scenario().type();
  if (scenario_type == Scenario::CRUISE ||
      scenario_type == Scenario::CRUISE_URBAN ||
      scenario_type == Scenario::CRUISE_HIGHWAY) {
    PrioritizeObstaclesForCruiseScenario(
        environment_features,
        std::dynamic_pointer_cast<CruiseScenarioFeatures>(
            ptr_scenario_features),
        obstacles_container);
  } else if (scenario_type == Scenario::JUNCTION ||
             scenario_type == Scenario::JUNCTION_TRAFFIC_LIGHT ||
             scenario_type == Scenario::JUNCTION_STOP_SIGN) {
    // TODO(kechxu) PrioritizeObstaclesInJunction
  }
}

void ScenarioManager::PrioritizeObstaclesForCruiseScenario(
    const EnvironmentFeatures& environment_features,
    const std::shared_ptr<CruiseScenarioFeatures> cruise_scenario_features,
    ObstaclesContainer* ptr_obstacle_contrainer) {
  const auto& obstacle_ids =
      ptr_obstacle_contrainer->GetCurrentFramePredictableObstacleIds();

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
