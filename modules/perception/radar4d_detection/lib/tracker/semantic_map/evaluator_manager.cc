/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/radar4d_detection/lib/tracker/semantic_map/evaluator_manager.h"

#include <algorithm>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/prediction/common/feature_output.h"
#include "modules/prediction/common/semantic_map.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"

namespace apollo {
namespace perception {

using apollo::perception::PerceptionObstacle;
using apollo::prediction::Feature;
using apollo::prediction::Obstacle;
using apollo::prediction::ObstacleHistory;
using apollo::prediction::ObstaclePriority;
using apollo::prediction::ObstaclesContainer;
using apollo::prediction::SemanticLSTMEvaluator;
using apollo::prediction::SemanticMap;
using IdObstacleListMap = std::unordered_map<int, std::list<Obstacle*>>;

bool IsTrainable(const Feature& feature) {
  if (feature.id() == FLAGS_ego_vehicle_id) {
    return false;
  }
  if (feature.priority().priority() == ObstaclePriority::IGNORE ||
      feature.is_still() || feature.type() != PerceptionObstacle::VEHICLE) {
    return false;
  }
  return true;
}

void EvaluatorManager::Init() {
  semantic_map_.reset(new SemanticMap());
  semantic_map_->Init();
  evaluator_.reset(new SemanticLSTMEvaluator(semantic_map_.get()));
  AINFO << "Init SemanticMap instance.";
}

void EvaluatorManager::Run(ObstaclesContainer* obstacles_container) {
  BuildObstacleIdHistoryMap(obstacles_container);
  semantic_map_->RunCurrFrame(obstacle_id_history_map_);
  AINFO << "starting evaluating objects in semantic map";
  std::vector<Obstacle*> dynamic_env;
  for (int id : obstacles_container->curr_frame_considered_obstacle_ids()) {
    Obstacle* obstacle = obstacles_container->GetObstacle(id);
    if (obstacle == nullptr) {
      continue;
    }
    if (obstacle->IsStill()) {
      ADEBUG << "Ignore still obstacle [" << id << "] in evaluator_manager";
      continue;
    }
    EvaluateObstacle(obstacle, obstacles_container, dynamic_env);
  }
}

void EvaluatorManager::EvaluateObstacle(Obstacle* obstacle,
                                        ObstaclesContainer* obstacles_container,
                                        std::vector<Obstacle*> dynamic_env) {
  // Select different evaluators depending on the obstacle's type.
  switch (obstacle->type()) {
    case PerceptionObstacle::VEHICLE: {
      evaluator_->Evaluate(obstacle, obstacles_container);
      break;
    }
    default:
      break;
  }
}

void EvaluatorManager::EvaluateObstacle(
    Obstacle* obstacle, ObstaclesContainer* obstacles_container) {
  std::vector<Obstacle*> dummy_dynamic_env;
  EvaluateObstacle(obstacle, obstacles_container, dummy_dynamic_env);
}

void EvaluatorManager::BuildObstacleIdHistoryMap(
    ObstaclesContainer* obstacles_container) {
  obstacle_id_history_map_.clear();
  std::vector<int> obstacle_ids =
      obstacles_container->curr_frame_movable_obstacle_ids();
  obstacle_ids.push_back(FLAGS_ego_vehicle_id);
  for (int id : obstacle_ids) {
    Obstacle* obstacle = obstacles_container->GetObstacle(id);
    if (obstacle == nullptr || obstacle->history_size() == 0) {
      continue;
    }
    size_t num_frames =
        std::min(static_cast<size_t>(10), obstacle->history_size());
    for (size_t i = 0; i < num_frames; ++i) {
      const Feature& obstacle_feature = obstacle->feature(i);
      Feature feature;
      feature.set_id(obstacle_feature.id());
      feature.set_timestamp(obstacle_feature.timestamp());
      feature.mutable_position()->CopyFrom(obstacle_feature.position());
      feature.set_theta(obstacle_feature.velocity_heading());
      if (obstacle_feature.id() != FLAGS_ego_vehicle_id) {
        feature.mutable_polygon_point()->CopyFrom(
            obstacle_feature.polygon_point());
        feature.set_length(obstacle_feature.length());
        feature.set_width(obstacle_feature.width());
      } else {
        const auto& vehicle_config =
            common::VehicleConfigHelper::Instance()->GetConfig();
        feature.set_length(vehicle_config.vehicle_param().length());
        feature.set_width(vehicle_config.vehicle_param().width());
      }
      obstacle_id_history_map_[id].add_feature()->CopyFrom(feature);
    }
    obstacle_id_history_map_[id].set_is_trainable(
        IsTrainable(obstacle->latest_feature()));
  }
}

}  // namespace perception
}  // namespace apollo
