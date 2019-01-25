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

#include <vector>

#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"
#include "modules/prediction/evaluator/vehicle/cost_evaluator.h"
#include "modules/prediction/evaluator/vehicle/cruise_mlp_evaluator.h"
#include "modules/prediction/evaluator/vehicle/junction_mlp_evaluator.h"
#include "modules/prediction/evaluator/vehicle/lane_scanning_evaluator.h"
#include "modules/prediction/evaluator/vehicle/mlp_evaluator.h"
#include "modules/prediction/evaluator/vehicle/rnn_evaluator.h"
#include "modules/prediction/evaluator/cyclist/cyclist_keep_lane_evaluator.h"

namespace apollo {
namespace prediction {

using apollo::common::adapter::AdapterConfig;
using apollo::perception::PerceptionObstacle;

EvaluatorManager::EvaluatorManager() { RegisterEvaluators(); }

void EvaluatorManager::RegisterEvaluators() {
  RegisterEvaluator(ObstacleConf::MLP_EVALUATOR);
  RegisterEvaluator(ObstacleConf::RNN_EVALUATOR);
  RegisterEvaluator(ObstacleConf::COST_EVALUATOR);
  RegisterEvaluator(ObstacleConf::CRUISE_MLP_EVALUATOR);
  RegisterEvaluator(ObstacleConf::JUNCTION_MLP_EVALUATOR);
  RegisterEvaluator(ObstacleConf::CYCLIST_KEEP_LANE_EVALUATOR);
  RegisterEvaluator(ObstacleConf::LANE_SCANNING_EVALUATOR);
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

    if (obstacle_conf.has_obstacle_status()) {
      if (obstacle_conf.obstacle_status() == ObstacleConf::ON_LANE) {
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
      } else if (obstacle_conf.obstacle_status() == ObstacleConf::IN_JUNCTION) {
        switch (obstacle_conf.obstacle_type()) {
          case PerceptionObstacle::VEHICLE: {
            vehicle_in_junction_evaluator_ = obstacle_conf.evaluator_type();
            break;
          }
          default: { break; }
        }
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

void EvaluatorManager::Run() {
  auto obstacles_container =
      ContainerManager::Instance()->GetContainer<ObstaclesContainer>(
          AdapterConfig::PERCEPTION_OBSTACLES);
  CHECK_NOTNULL(obstacles_container);

  std::vector<Obstacle*> dynamic_env;
  for (int id : obstacles_container->curr_frame_predictable_obstacle_ids()) {
    if (id < 0) {
      ADEBUG << "The obstacle has invalid id [" << id << "].";
      continue;
    }
    Obstacle* obstacle = obstacles_container->GetObstacle(id);

    if (obstacle == nullptr) {
      continue;
    }
    if (obstacle->ToIgnore() || obstacle->IsStill()) {
      ADEBUG << "Ignore obstacle [" << id << "] in evaluator_manager";
      continue;
    }

    EvaluateObstacle(obstacle, dynamic_env);
  }
}

void EvaluatorManager::EvaluateObstacle(
    Obstacle* obstacle, std::vector<Obstacle*> dynamic_env) {
  Evaluator* evaluator = nullptr;
  // Select different evaluators depending on the obstacle's type.
  switch (obstacle->type()) {
    case PerceptionObstacle::VEHICLE: {
      if (obstacle->HasJunctionFeatureWithExits() &&
          !obstacle->IsCloseToJunctionExit()) {
        evaluator = GetEvaluator(vehicle_in_junction_evaluator_);
        CHECK_NOTNULL(evaluator);
      } else if (obstacle->IsOnLane()) {
        evaluator = GetEvaluator(vehicle_on_lane_evaluator_);
        CHECK_NOTNULL(evaluator);
      } else {
        ADEBUG << "Obstacle: " << obstacle->id() << " is neither "
               "on lane, nor in junction. Skip evaluating.";
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

  // Evaluate using the selected evaluator.
  if (evaluator != nullptr) {
    if (evaluator->GetName() == "LANE_SCANNING_EVALUATOR") {
      // For evaluators that need surrounding obstacles' info.
      evaluator->Evaluate(obstacle, dynamic_env);
    } else {
      // For evaluators that don't need surrounding info.
      evaluator->Evaluate(obstacle);
    }
  }
}

void EvaluatorManager::EvaluateObstacle(Obstacle* obstacle) {
  std::vector<Obstacle*> dummy_dynamic_env;
  EvaluateObstacle(obstacle, dummy_dynamic_env);
}

std::unique_ptr<Evaluator> EvaluatorManager::CreateEvaluator(
    const ObstacleConf::EvaluatorType& type) {
  std::unique_ptr<Evaluator> evaluator_ptr(nullptr);
  switch (type) {
    case ObstacleConf::MLP_EVALUATOR: {
      evaluator_ptr.reset(new MLPEvaluator());
      break;
    }
    case ObstacleConf::CRUISE_MLP_EVALUATOR: {
      evaluator_ptr.reset(new CruiseMLPEvaluator());
      break;
    }
    case ObstacleConf::JUNCTION_MLP_EVALUATOR: {
      evaluator_ptr.reset(new JunctionMLPEvaluator());
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
    case ObstacleConf::CYCLIST_KEEP_LANE_EVALUATOR: {
      evaluator_ptr.reset(new CyclistKeepLaneEvaluator());
      break;
    }
    case ObstacleConf::LANE_SCANNING_EVALUATOR: {
      evaluator_ptr.reset(new LaneScanningEvaluator());
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
