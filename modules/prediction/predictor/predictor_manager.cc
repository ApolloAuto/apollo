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

#include <memory>

#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"
#include "modules/prediction/predictor/free_move/free_move_predictor.h"
#include "modules/prediction/predictor/lane_sequence/lane_sequence_predictor.h"
#include "modules/prediction/predictor/move_sequence/move_sequence_predictor.h"
#include "modules/prediction/predictor/regional/regional_predictor.h"

namespace apollo {
namespace prediction {

using apollo::common::adapter::AdapterConfig;
using apollo::perception::PerceptionObstacle;
using apollo::perception::PerceptionObstacles;

PredictorManager::PredictorManager() { RegisterPredictors(); }

void PredictorManager::RegisterPredictors() {
  RegisterPredictor(ObstacleConf::LANE_SEQUENCE_PREDICTOR);
  RegisterPredictor(ObstacleConf::MOVE_SEQUENCE_PREDICTOR);
  RegisterPredictor(ObstacleConf::FREE_MOVE_PREDICTOR);
  RegisterPredictor(ObstacleConf::REGIONAL_PREDICTOR);
}

void PredictorManager::Init(const PredictionConf& config) {
  for (const auto& obstacle_conf : config.obstacle_conf()) {
    if (!obstacle_conf.has_obstacle_type()) {
      ADEBUG << "Obstacle config [" << obstacle_conf.ShortDebugString()
             << "] has not defined obstacle type.";
      continue;
    }

    if (obstacle_conf.obstacle_type() == PerceptionObstacle::VEHICLE) {
      if (!obstacle_conf.has_obstacle_status() ||
          !obstacle_conf.has_predictor_type()) {
        ADEBUG << "Vehicle obstacle config ["
               << obstacle_conf.ShortDebugString()
               << "] has not defined obstacle status or predictor type.";
        continue;
      } else if (obstacle_conf.obstacle_status() == ObstacleConf::ON_LANE) {
        vehicle_on_lane_predictor_ = obstacle_conf.predictor_type();
      } else if (obstacle_conf.obstacle_status() == ObstacleConf::OFF_LANE) {
        vehicle_off_lane_predictor_ = obstacle_conf.predictor_type();
      }
    } else if (obstacle_conf.obstacle_type() ==
               PerceptionObstacle::PEDESTRIAN) {
      pedestrian_predictor_ = obstacle_conf.predictor_type();
    }
  }

  AINFO << "Defined vehicle on lane obstacle predictor ["
        << vehicle_on_lane_predictor_ << "].";
  AINFO << "Defined vehicle off lane obstacle predictor ["
        << vehicle_off_lane_predictor_ << "].";
  AINFO << "Defined pedestrian obstacle predictor [" << pedestrian_predictor_
        << "].";
  AINFO << "Defined default obstacle predictor [" << default_predictor_ << "].";
}

Predictor* PredictorManager::GetPredictor(
    const ObstacleConf::PredictorType& type) {
  auto it = predictors_.find(type);
  return it != predictors_.end() ? it->second.get() : nullptr;
}

void PredictorManager::Run(const PerceptionObstacles& perception_obstacles) {
  prediction_obstacles_.Clear();
  ObstaclesContainer* container = dynamic_cast<ObstaclesContainer*>(
      ContainerManager::instance()->GetContainer(
          AdapterConfig::PERCEPTION_OBSTACLES));
  CHECK_NOTNULL(container);

  Predictor* predictor = nullptr;
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

    PredictionObstacle prediction_obstacle;
    prediction_obstacle.set_timestamp(perception_obstacle.timestamp());
    Obstacle* obstacle = container->GetObstacle(id);
    if (obstacle != nullptr) {
      switch (perception_obstacle.type()) {
        case PerceptionObstacle::VEHICLE: {
          if (obstacle->IsOnLane()) {
            predictor = GetPredictor(vehicle_on_lane_predictor_);
          } else {
            predictor = GetPredictor(vehicle_off_lane_predictor_);
          }
          break;
        }
        case PerceptionObstacle::PEDESTRIAN: {
          predictor = GetPredictor(pedestrian_predictor_);
          break;
        }
        case PerceptionObstacle::BICYCLE: {
          if (obstacle->IsOnLane() && !obstacle->IsNearJunction()) {
            predictor = GetPredictor(cyclist_on_lane_predictor_);
          } else {
            predictor = GetPredictor(cyclist_off_lane_predictor_);
          }
          break;
        }
        default: {
          if (obstacle->IsOnLane()) {
            predictor = GetPredictor(vehicle_on_lane_predictor_);
          } else {
            predictor = GetPredictor(vehicle_off_lane_predictor_);
          }
          break;
        }
      }

      if (predictor != nullptr) {
        predictor->Predict(obstacle);
        for (const auto& trajectory : predictor->trajectories()) {
          prediction_obstacle.add_trajectory()->CopyFrom(trajectory);
        }
      }
      prediction_obstacle.set_timestamp(obstacle->timestamp());
    }

    prediction_obstacle.set_predicted_period(FLAGS_prediction_duration);
    prediction_obstacle.mutable_perception_obstacle()->CopyFrom(
        perception_obstacle);

    prediction_obstacles_.add_prediction_obstacle()->CopyFrom(
        prediction_obstacle);
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
    case ObstacleConf::MOVE_SEQUENCE_PREDICTOR: {
      predictor_ptr.reset(new MoveSequencePredictor());
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
    default: { break; }
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
