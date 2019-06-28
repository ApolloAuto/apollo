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

#include <list>
#include <memory>
#include <unordered_map>

#include "modules/prediction/common/feature_output.h"
#include "modules/prediction/common/prediction_constants.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_system_gflags.h"
#include "modules/prediction/common/prediction_thread_pool.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/predictor/empty/empty_predictor.h"
#include "modules/prediction/predictor/extrapolation/extrapolation_predictor.h"
#include "modules/prediction/predictor/free_move/free_move_predictor.h"
#include "modules/prediction/predictor/interaction/interaction_predictor.h"
#include "modules/prediction/predictor/junction/junction_predictor.h"
#include "modules/prediction/predictor/lane_sequence/lane_sequence_predictor.h"
#include "modules/prediction/predictor/move_sequence/move_sequence_predictor.h"
#include "modules/prediction/predictor/regional/regional_predictor.h"
#include "modules/prediction/predictor/single_lane/single_lane_predictor.h"

namespace apollo {
namespace prediction {

using apollo::common::adapter::AdapterConfig;
using apollo::perception::PerceptionObstacle;
using IdObstacleListMap = std::unordered_map<int, std::list<Obstacle*>>;
using IdPredictionObstacleMap =
    std::unordered_map<int, std::shared_ptr<PredictionObstacle>>;

namespace {

void GroupObstaclesByObstacleId(const int obstacle_id,
                                ObstaclesContainer* const obstacles_container,
                                IdObstacleListMap* const id_obstacle_map) {
  Obstacle* obstacle_ptr = obstacles_container->GetObstacle(obstacle_id);
  if (obstacle_ptr == nullptr) {
    AERROR << "Null obstacle [" << obstacle_id << "] found";
    return;
  }
  int id_mod = obstacle_id % FLAGS_max_thread_num;
  (*id_obstacle_map)[id_mod].push_back(obstacle_ptr);
}

}  // namespace

PredictorManager::PredictorManager() { RegisterPredictors(); }

void PredictorManager::RegisterPredictors() {
  RegisterPredictor(ObstacleConf::LANE_SEQUENCE_PREDICTOR);
  RegisterPredictor(ObstacleConf::MOVE_SEQUENCE_PREDICTOR);
  RegisterPredictor(ObstacleConf::SINGLE_LANE_PREDICTOR);
  RegisterPredictor(ObstacleConf::FREE_MOVE_PREDICTOR);
  RegisterPredictor(ObstacleConf::REGIONAL_PREDICTOR);
  RegisterPredictor(ObstacleConf::EMPTY_PREDICTOR);
  RegisterPredictor(ObstacleConf::JUNCTION_PREDICTOR);
  RegisterPredictor(ObstacleConf::EXTRAPOLATION_PREDICTOR);
  RegisterPredictor(ObstacleConf::INTERACTION_PREDICTOR);
}

void PredictorManager::Init(const PredictionConf& config) {
  for (const auto& obstacle_conf : config.obstacle_conf()) {
    if (!obstacle_conf.has_obstacle_type()) {
      AERROR << "Obstacle config [" << obstacle_conf.ShortDebugString()
             << "] has not defined obstacle type.";
      continue;
    }

    if (!obstacle_conf.has_predictor_type()) {
      AERROR << "Obstacle config [" << obstacle_conf.ShortDebugString()
             << "] has not defined predictor type.";
      continue;
    }

    switch (obstacle_conf.obstacle_type()) {
      case PerceptionObstacle::VEHICLE: {
        if (obstacle_conf.has_obstacle_status()) {
          if (obstacle_conf.obstacle_status() == ObstacleConf::ON_LANE) {
            vehicle_on_lane_predictor_ = obstacle_conf.predictor_type();
          } else if (obstacle_conf.obstacle_status() ==
                     ObstacleConf::OFF_LANE) {
            vehicle_off_lane_predictor_ = obstacle_conf.predictor_type();
          } else if (obstacle_conf.obstacle_status() ==
                     ObstacleConf::IN_JUNCTION) {
            vehicle_in_junction_predictor_ = obstacle_conf.predictor_type();
          }
        }
        break;
      }
      case PerceptionObstacle::BICYCLE: {
        if (obstacle_conf.has_obstacle_status()) {
          if (obstacle_conf.obstacle_status() == ObstacleConf::ON_LANE) {
            cyclist_on_lane_predictor_ = obstacle_conf.predictor_type();
          } else if (obstacle_conf.obstacle_status() ==
                     ObstacleConf::OFF_LANE) {
            cyclist_off_lane_predictor_ = obstacle_conf.predictor_type();
          }
        }
        break;
      }
      case PerceptionObstacle::PEDESTRIAN: {
        pedestrian_predictor_ = obstacle_conf.predictor_type();
        break;
      }
      case PerceptionObstacle::UNKNOWN: {
        if (obstacle_conf.has_obstacle_status()) {
          if (obstacle_conf.obstacle_status() == ObstacleConf::ON_LANE) {
            default_on_lane_predictor_ = obstacle_conf.predictor_type();
          } else if (obstacle_conf.obstacle_status() ==
                     ObstacleConf::OFF_LANE) {
            default_off_lane_predictor_ = obstacle_conf.predictor_type();
          }
        }
        break;
      }
      default: { break; }
    }
  }

  AINFO << "Defined vehicle on lane obstacle predictor ["
        << vehicle_on_lane_predictor_ << "].";
  AINFO << "Defined vehicle off lane obstacle predictor ["
        << vehicle_off_lane_predictor_ << "].";
  AINFO << "Defined bicycle on lane obstacle predictor ["
        << cyclist_on_lane_predictor_ << "].";
  AINFO << "Defined bicycle off lane obstacle predictor ["
        << cyclist_off_lane_predictor_ << "].";
  AINFO << "Defined pedestrian obstacle predictor [" << pedestrian_predictor_
        << "].";
  AINFO << "Defined default on lane obstacle predictor ["
        << default_on_lane_predictor_ << "].";
  AINFO << "Defined default off lane obstacle predictor ["
        << default_off_lane_predictor_ << "].";
}

Predictor* PredictorManager::GetPredictor(
    const ObstacleConf::PredictorType& type) {
  auto it = predictors_.find(type);
  return it != predictors_.end() ? it->second.get() : nullptr;
}

void PredictorManager::Run() {
  prediction_obstacles_.Clear();
  auto obstacles_container =
      ContainerManager::Instance()->GetContainer<ObstaclesContainer>(
          AdapterConfig::PERCEPTION_OBSTACLES);

  auto adc_trajectory_container =
      ContainerManager::Instance()->GetContainer<ADCTrajectoryContainer>(
          AdapterConfig::PLANNING_TRAJECTORY);

  CHECK_NOTNULL(obstacles_container);

  if (FLAGS_enable_multi_thread) {
    PredictObstaclesInParallel(obstacles_container, adc_trajectory_container);
  } else {
    PredictObstacles(obstacles_container, adc_trajectory_container);
  }
}

void PredictorManager::PredictObstacles(
    ObstaclesContainer* obstacles_container,
    ADCTrajectoryContainer* adc_trajectory_container) {
  for (const int id : obstacles_container->curr_frame_obstacle_ids()) {
    if (id < 0) {
      ADEBUG << "The obstacle has invalid id [" << id << "].";
      continue;
    }

    PredictionObstacle prediction_obstacle;
    Obstacle* obstacle = obstacles_container->GetObstacle(id);

    PerceptionObstacle perception_obstacle =
        obstacles_container->GetPerceptionObstacle(id);
    // if obstacle == nullptr, that means obstacle is unmovable
    // Checkout the logic of unmovable in obstacle.cc
    if (obstacle != nullptr) {
      PredictObstacle(obstacle, &prediction_obstacle, adc_trajectory_container);
    } else {  // obstacle == nullptr
      prediction_obstacle.set_timestamp(perception_obstacle.timestamp());
      prediction_obstacle.set_is_static(true);
    }

    prediction_obstacle.set_predicted_period(
        FLAGS_prediction_trajectory_time_length);
    prediction_obstacle.mutable_perception_obstacle()->CopyFrom(
        perception_obstacle);

    prediction_obstacles_.add_prediction_obstacle()->CopyFrom(
        prediction_obstacle);
  }
}

void PredictorManager::PredictObstaclesInParallel(
    ObstaclesContainer* obstacles_container,
    ADCTrajectoryContainer* adc_trajectory_container) {
  IdPredictionObstacleMap id_prediction_obstacle_map;
  for (int id : obstacles_container->curr_frame_obstacle_ids()) {
    id_prediction_obstacle_map[id] = std::make_shared<PredictionObstacle>();
  }
  IdObstacleListMap id_obstacle_map;
  for (int id : obstacles_container->curr_frame_obstacle_ids()) {
    Obstacle* obstacle = obstacles_container->GetObstacle(id);
    const PerceptionObstacle& perception_obstacle =
        obstacles_container->GetPerceptionObstacle(id);
    if (obstacle == nullptr) {
      std::shared_ptr<PredictionObstacle> prediction_obstacle_ptr =
          id_prediction_obstacle_map[id];
      prediction_obstacle_ptr->set_is_static(true);
      prediction_obstacle_ptr->set_timestamp(perception_obstacle.timestamp());
    } else {
      GroupObstaclesByObstacleId(id, obstacles_container, &id_obstacle_map);
    }
  }
  PredictionThreadPool::ForEach(
      id_obstacle_map.begin(), id_obstacle_map.end(),
      [&](IdObstacleListMap::iterator::value_type& obstacles_iter) {
        for (auto obstacle_ptr : obstacles_iter.second) {
          int id = obstacle_ptr->id();
          PredictObstacle(obstacle_ptr, id_prediction_obstacle_map[id].get(),
                          adc_trajectory_container);
        }
      });
  for (auto& item : id_prediction_obstacle_map) {
    int id = item.first;
    auto prediction_obstacle_ptr = item.second;
    const PerceptionObstacle& perception_obstacle =
        obstacles_container->GetPerceptionObstacle(id);
    prediction_obstacle_ptr->set_predicted_period(
        FLAGS_prediction_trajectory_time_length);
    prediction_obstacle_ptr->mutable_perception_obstacle()->CopyFrom(
        perception_obstacle);
    prediction_obstacles_.add_prediction_obstacle()->CopyFrom(
        *prediction_obstacle_ptr);
  }
}

void PredictorManager::PredictObstacle(
    Obstacle* obstacle, PredictionObstacle* const prediction_obstacle,
    ADCTrajectoryContainer* adc_trajectory_container) {
  CHECK_NOTNULL(obstacle);
  Predictor* predictor = nullptr;
  prediction_obstacle->set_timestamp(obstacle->timestamp());
  if (obstacle->ToIgnore()) {
    ADEBUG << "Ignore obstacle [" << obstacle->id() << "]";
    predictor = GetPredictor(ObstacleConf::EMPTY_PREDICTOR);
    prediction_obstacle->mutable_priority()->set_priority(
        ObstaclePriority::IGNORE);
  } else if (obstacle->IsStill()) {
    ADEBUG << "Still obstacle [" << obstacle->id() << "]";
    predictor = GetPredictor(ObstacleConf::EMPTY_PREDICTOR);
  } else {
    switch (obstacle->type()) {
      case PerceptionObstacle::VEHICLE: {
        if (!obstacle->IsOnLane()) {
          predictor = GetPredictor(vehicle_off_lane_predictor_);
          CHECK_NOTNULL(predictor);
        } else if (obstacle->HasJunctionFeatureWithExits() &&
                   !obstacle->IsCloseToJunctionExit()) {
          predictor = GetPredictor(vehicle_in_junction_predictor_);
          CHECK_NOTNULL(predictor);
        } else {
          predictor = GetPredictor(vehicle_on_lane_predictor_);
          CHECK_NOTNULL(predictor);
        }
        break;
      }
      case PerceptionObstacle::PEDESTRIAN: {
        predictor = GetPredictor(pedestrian_predictor_);
        break;
      }
      case PerceptionObstacle::BICYCLE: {
        if (obstacle->IsOnLane()) {
          predictor = GetPredictor(cyclist_on_lane_predictor_);
          // TODO(kechxu) add a specific predictor in junction
        } else {
          predictor = GetPredictor(cyclist_off_lane_predictor_);
        }
        break;
      }
      default: {
        if (obstacle->IsOnLane()) {
          predictor = GetPredictor(default_on_lane_predictor_);
        } else {
          predictor = GetPredictor(default_off_lane_predictor_);
        }
        break;
      }
    }
  }

  if (predictor != nullptr) {
    predictor->Predict(obstacle);
    if (FLAGS_enable_trim_prediction_trajectory &&
        obstacle->type() == PerceptionObstacle::VEHICLE) {
      CHECK_NOTNULL(adc_trajectory_container);
      predictor->TrimTrajectories(*adc_trajectory_container, obstacle);
    }
  }
  for (const auto& trajectory :
       obstacle->latest_feature().predicted_trajectory()) {
    prediction_obstacle->add_trajectory()->CopyFrom(trajectory);
  }
  prediction_obstacle->set_timestamp(obstacle->timestamp());
  prediction_obstacle->mutable_priority()->CopyFrom(
      obstacle->latest_feature().priority());
  prediction_obstacle->set_is_static(obstacle->IsStill());
  if (FLAGS_prediction_offline_mode ==
      PredictionConstants::kDumpPredictionResult) {
    FeatureOutput::InsertPredictionResult(obstacle->id(), *prediction_obstacle,
                                          obstacle->obstacle_conf());
  }
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
    case ObstacleConf::SINGLE_LANE_PREDICTOR: {
      predictor_ptr.reset(new SingleLanePredictor());
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
    case ObstacleConf::JUNCTION_PREDICTOR: {
      predictor_ptr.reset(new JunctionPredictor());
      break;
    }
    case ObstacleConf::EXTRAPOLATION_PREDICTOR: {
      predictor_ptr.reset(new ExtrapolationPredictor());
      break;
    }
    case ObstacleConf::INTERACTION_PREDICTOR: {
      predictor_ptr.reset(new InteractionPredictor());
      break;
    }
    case ObstacleConf::EMPTY_PREDICTOR: {
      predictor_ptr.reset(new EmptyPredictor());
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
