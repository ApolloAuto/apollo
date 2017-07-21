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

#include "modules/prediction/predictor/predictor_manager.h"
#include "modules/prediction/predictor/predictor_factory.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"

namespace apollo {
namespace prediction {

using ::apollo::perception::PerceptionObstacles;
using ::apollo::perception::PerceptionObstacle;

PredictorManager::PredictorManager() {
  PredictorFactory::instance()->RegisterPredictor();
}

Predictor* PredictorManager::GetPredictor(
    const ObstacleConf::PredictorType& type) {
  return PredictorFactory::instance()->CreatePredictor(type).get();
}

void PredictorManager::Run(
    const PerceptionObstacles& perception_obstacles) {
  prediction_obstacles_.Clear();
  ObstaclesContainer *container = dynamic_cast<ObstaclesContainer*>(
      ContainerManager::instance()->mutable_container("ObstaclesContainer"));
  CHECK_NOTNULL(container);

  Predictor *predictor = nullptr;
  for (const auto& perception_obstacle :
      perception_obstacles.perception_obstacle()) {
    int id = perception_obstacle.id();
    switch (perception_obstacle.type()) {
      case PerceptionObstacle::VEHICLE: {
        predictor = GetPredictor(ObstacleConf::LANE_SEQUENCE_PREDICTOR);
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
    if (predictor != nullptr) {
      Obstacle* obstacle = container->GetObstacle(id);
      predictor->Predict(obstacle);
      prediction_obstacles_.add_prediction_obstacle()->CopyFrom(
        predictor->prediction_obstacle());
    }
  }
}

const PredictionObstacles& PredictorManager::prediction_obstacles() {
  return prediction_obstacles_;
}

}  // namespace prediction
}  // namespace apollo

