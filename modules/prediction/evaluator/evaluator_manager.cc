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

#include "modules/prediction/evaluator/vehicle/mlp_evaluator.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"
#include "modules/common/log.h"

namespace apollo {
namespace prediction {

using ::apollo::perception::PerceptionObstacles;
using ::apollo::perception::PerceptionObstacle;
using ::apollo::common::adapter::AdapterConfig;

EvaluatorManager::EvaluatorManager() {
  RegisterEvaluators();
}

void EvaluatorManager::RegisterEvaluators() {
  RegisterEvaluator(ObstacleConf::MLP_EVALUATOR);
}

void EvaluatorManager::Init(const PredictionConf& config) {
  
}

Evaluator* EvaluatorManager::GetEvaluator(
    const ObstacleConf::EvaluatorType& type) {
  if (evaluators_.find(type) != evaluators_.end()) {
    return evaluators_[type].get();
  } else {
    return nullptr;
  }
}

void EvaluatorManager::Run(
    const ::apollo::perception::PerceptionObstacles& perception_obstacles) {
  AINFO << "Start run evaluator manager";
  ObstaclesContainer *container = dynamic_cast<ObstaclesContainer*>(
      ContainerManager::instance()->GetContainer(
      AdapterConfig::PERCEPTION_OBSTACLES));
  CHECK_NOTNULL(container);
  AINFO << "Start for loop";

  Evaluator *evaluator = nullptr;
  for (const auto& perception_obstacle :
      perception_obstacles.perception_obstacle()) {
    int id = perception_obstacle.id();
    Obstacle* obstacle = container->GetObstacle(id);
    CHECK_NOTNULL(obstacle);
    switch (perception_obstacle.type()) {
      case PerceptionObstacle::VEHICLE: {
        if (obstacle->IsOnLane()) {
          evaluator = GetEvaluator(ObstacleConf::MLP_EVALUATOR);
          CHECK_NOTNULL(evaluator);
        }
        break;
      }
      default: {
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
    default: {
      break;
    }
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
