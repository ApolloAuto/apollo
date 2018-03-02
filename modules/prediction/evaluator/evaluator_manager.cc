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

#include "modules/prediction/evaluator/evaluator_manager.h"

#include "modules/common/log.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"
#include "modules/prediction/evaluator/vehicle/mlp_evaluator.h"
#include "modules/prediction/evaluator/vehicle/rnn_evaluator.h"
#include "modules/prediction/evaluator/vehicle/cost_evaluator.h"

namespace apollo {
namespace prediction {

using apollo::common::adapter::AdapterConfig;
using apollo::perception::PerceptionObstacle;
using apollo::perception::PerceptionObstacles;

EvaluatorManager::EvaluatorManager() { RegisterEvaluators(); }

void EvaluatorManager::RegisterEvaluators() {
  RegisterEvaluator(ObstacleConf::MLP_EVALUATOR);
  RegisterEvaluator(ObstacleConf::RNN_EVALUATOR);
  RegisterEvaluator(ObstacleConf::COST_EVALUATOR);
}

void EvaluatorManager::Init(const PredictionConf& config) {
  for (const auto& obstacle_conf : config.obstacle_conf()) {
    if (!obstacle_conf.has_obstacle_type()) {
      AERROR << "Obstacle config [" << obstacle_conf.ShortDebugString()
             << "] has not defined obstacle type.";
      continue;
    }

    if (!obstacle_conf.has_evaluator_type()) {
      ADEBUG << "Obstacle config [" << obstacle_conf.ShortDebugString()
             << "] has not defined evaluator type.";
      continue;
    }

    if (obstacle_conf.has_obstacle_status() &&
        obstacle_conf.obstacle_status() == ObstacleConf::ON_LANE) {
      switch (obstacle_conf.obstacle_type()) {
        case PerceptionObstacle::VEHICLE: {
          vehicle_on_lane_evaluator_ = obstacle_conf.evaluator_type();
          break;
        }
        case PerceptionObstacle::BICYCLE: {
          cyclist_on_lane_evaluator_ = obstacle_conf.evaluator_type();
          break;
        }
        case PerceptionObstacle::PEDESTRIAN: {
          break;
        }
        case PerceptionObstacle::UNKNOWN: {
          default_on_lane_evaluator_ = obstacle_conf.evaluator_type();
          break;
        }
        default: { break; }
      }
    }
  }

  AINFO << "Defined vehicle on lane obstacle evaluator ["
        << vehicle_on_lane_evaluator_ << "]";
  AINFO << "Defined cyclist on lane obstacle evaluator ["
        << cyclist_on_lane_evaluator_ << "]";
  AINFO << "Defined default on lane obstacle evaluator ["
        << default_on_lane_evaluator_ << "]";
}

Evaluator* EvaluatorManager::GetEvaluator(
    const ObstacleConf::EvaluatorType& type) {
  auto it = evaluators_.find(type);
  return it != evaluators_.end() ? it->second.get() : nullptr;
}

void EvaluatorManager::Run(
    const perception::PerceptionObstacles& perception_obstacles) {
  ObstaclesContainer* container = dynamic_cast<ObstaclesContainer*>(
      ContainerManager::instance()->GetContainer(
          AdapterConfig::PERCEPTION_OBSTACLES));
  CHECK_NOTNULL(container);

  Evaluator* evaluator = nullptr;
  for (const auto& perception_obstacle :
       perception_obstacles.perception_obstacle()) {
    if (!perception_obstacle.has_id()) {
      AERROR << "A perception obstacle has no id.";
      continue;
    }

    int id = perception_obstacle.id();
    if (id < 0) {
      AERROR << "A perception obstacle has invalid id [" << id << "].";
      continue;
    }
    Obstacle* obstacle = container->GetObstacle(id);

    if (obstacle == nullptr) {
      continue;
    }

    switch (perception_obstacle.type()) {
      case PerceptionObstacle::VEHICLE: {
        if (obstacle->IsOnLane()) {
          evaluator = GetEvaluator(vehicle_on_lane_evaluator_);
          CHECK_NOTNULL(evaluator);
        }
        break;
      }
      case PerceptionObstacle::BICYCLE: {
        if (obstacle->IsOnLane()) {
          evaluator = GetEvaluator(cyclist_on_lane_evaluator_);
          CHECK_NOTNULL(evaluator);
        }
        break;
      }
      case PerceptionObstacle::PEDESTRIAN: {
        break;
      }
      default: {
        if (obstacle->IsOnLane()) {
          evaluator = GetEvaluator(default_on_lane_evaluator_);
          CHECK_NOTNULL(evaluator);
        }
        break;
      }
    }
    if (evaluator != nullptr) {
      evaluator->Evaluate(obstacle);
    }
  }
}

std::unique_ptr<Evaluator> EvaluatorManager::CreateEvaluator(
    const ObstacleConf::EvaluatorType& type) {
  std::unique_ptr<Evaluator> evaluator_ptr(nullptr);
  switch (type) {
    case ObstacleConf::MLP_EVALUATOR: {
      evaluator_ptr.reset(new MLPEvaluator());
      break;
    }
    case ObstacleConf::RNN_EVALUATOR: {
      evaluator_ptr.reset(new RNNEvaluator());
      break;
    }
    case ObstacleConf::COST_EVALUATOR: {
      evaluator_ptr.reset(new CostEvaluator());
      break;
    }
    default: { break; }
  }
  return evaluator_ptr;
}

void EvaluatorManager::RegisterEvaluator(
    const ObstacleConf::EvaluatorType& type) {
  evaluators_[type] = CreateEvaluator(type);
  AINFO << "Evaluator [" << type << "] is registered.";
}

}  // namespace prediction
}  // namespace apollo
