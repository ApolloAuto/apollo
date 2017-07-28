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

#include <memory>

#include "modules/prediction/predictor/predictor_manager.h"

#include "modules/prediction/predictor/vehicle/lane_sequence_predictor.h"
#include "modules/prediction/predictor/vehicle/free_move_predictor.h"
#include "modules/prediction/predictor/pedestrian/regional_predictor.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"

namespace apollo {
namespace prediction {

using ::apollo::perception::PerceptionObstacles;
using ::apollo::perception::PerceptionObstacle;
using ::apollo::common::adapter::AdapterConfig;

PredictorManager::PredictorManager() {
  RegisterPredictors();
}

void PredictorManager::RegisterPredictors() {
  RegisterPredictor(ObstacleConf::LANE_SEQUENCE_PREDICTOR);
  RegisterPredictor(ObstacleConf::FREE_MOVE_PREDICTOR);
  RegisterPredictor(ObstacleConf::REGIONAL_PREDICTOR);
}

void PredictorManager::Init(const PredictionConf& config) {

}

Predictor* PredictorManager::GetPredictor(
    const ObstacleConf::PredictorType& type) {
  if (predictors_.find(type) != predictors_.end()) {
    return predictors_[type].get();
  } else {
    return nullptr;
  }
}

void PredictorManager::Run(
    const PerceptionObstacles& perception_obstacles) {
  prediction_obstacles_.Clear();
  ObstaclesContainer *container = dynamic_cast<ObstaclesContainer*>(
      ContainerManager::instance()->GetContainer(
      AdapterConfig::PERCEPTION_OBSTACLES));
  CHECK_NOTNULL(container);

  Predictor *predictor = nullptr;
  for (const auto& perception_obstacle :
      perception_obstacles.perception_obstacle()) {
    int id = perception_obstacle.id();
    Obstacle* obstacle = container->GetObstacle(id);
    CHECK_NOTNULL(obstacle);
    switch (perception_obstacle.type()) {
      case PerceptionObstacle::VEHICLE: {
        if (obstacle->IsOnLane()) {
          predictor = GetPredictor(ObstacleConf::LANE_SEQUENCE_PREDICTOR);
        } else {
          predictor = GetPredictor(ObstacleConf::FREE_MOVE_PREDICTOR);
        }
        break;
      }
      case PerceptionObstacle::PEDESTRIAN: {
        predictor = GetPredictor(ObstacleConf::FREE_MOVE_PREDICTOR);
        break;
      }
      default: {
        predictor = GetPredictor(ObstacleConf::FREE_MOVE_PREDICTOR);
        break;
      }
    }
    PredictionObstacle prediction_obstacle;
    if (predictor != nullptr) {
      predictor->Predict(obstacle);
      prediction_obstacle.CopyFrom(predictor->prediction_obstacle());
    }
    prediction_obstacle.mutable_perception_obstacle()->
        CopyFrom(perception_obstacle);
    prediction_obstacles_.add_prediction_obstacle()->
        CopyFrom(prediction_obstacle);
  }
  prediction_obstacles_.set_perception_error_code(
      perception_obstacles.error_code());
}

std::unique_ptr<Predictor> PredictorManager::CreatePredictor(
    const ObstacleConf::PredictorType& type) {
  std::unique_ptr<Predictor> predictor_ptr(nullptr);
  switch (type) {
    case ObstacleConf::LANE_SEQUENCE_PREDICTOR: {
      predictor_ptr.reset(new LaneSequencePredictor());
      break;
    }
    case ObstacleConf::FREE_MOVE_PREDICTOR: {
      predictor_ptr.reset(new FreeMovePredictor());
      break;
    }
    case ObstacleConf::REGIONAL_PREDICTOR: {
      predictor_ptr.reset(new RegionalPredictor());
      break;
    }
    default: {
      break;
    }
  }
  return predictor_ptr;
}

void PredictorManager::RegisterPredictor(
    const ObstacleConf::PredictorType& type) {
  predictors_[type] = CreatePredictor(type);
  AINFO << "Predictor [" << type << "] is registered.";
}

const PredictionObstacles& PredictorManager::prediction_obstacles() {
  return prediction_obstacles_;
}

}  // namespace prediction
}  // namespace apollo

