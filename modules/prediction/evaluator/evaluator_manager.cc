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

#include <algorithm>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/prediction/common/feature_output.h"
#include "modules/prediction/common/prediction_constants.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_system_gflags.h"
#include "modules/prediction/common/prediction_thread_pool.h"
#include "modules/prediction/common/semantic_map.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"
#include "modules/prediction/evaluator/cyclist/cyclist_keep_lane_evaluator.h"
#include "modules/prediction/evaluator/pedestrian/pedestrian_interaction_evaluator.h"
#include "modules/prediction/evaluator/vehicle/cost_evaluator.h"
#include "modules/prediction/evaluator/vehicle/cruise_mlp_evaluator.h"
#include "modules/prediction/evaluator/vehicle/junction_map_evaluator.h"
#include "modules/prediction/evaluator/vehicle/junction_mlp_evaluator.h"
#include "modules/prediction/evaluator/vehicle/lane_aggregating_evaluator.h"
#include "modules/prediction/evaluator/vehicle/lane_scanning_evaluator.h"
#include "modules/prediction/evaluator/vehicle/mlp_evaluator.h"
#include "modules/prediction/evaluator/vehicle/semantic_lstm_evaluator.h"

namespace apollo {
namespace prediction {

using apollo::perception::PerceptionObstacle;
using IdObstacleListMap = std::unordered_map<int, std::list<Obstacle*>>;

namespace {

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

void GroupObstaclesByObstacleIds(ObstaclesContainer* const obstacles_container,
                                 IdObstacleListMap* const id_obstacle_map) {
  int caution_thread_idx = 0;
  for (int obstacle_id :
       obstacles_container->curr_frame_considered_obstacle_ids()) {
    Obstacle* obstacle_ptr = obstacles_container->GetObstacle(obstacle_id);
    if (obstacle_ptr == nullptr) {
      AERROR << "Null obstacle [" << obstacle_id << "] found";
      continue;
    }
    if (obstacle_ptr->IsStill()) {
      ADEBUG << "Ignore still obstacle [" << obstacle_id << "]";
      continue;
    }
    const Feature& feature = obstacle_ptr->latest_feature();
    if (feature.priority().priority() == ObstaclePriority::IGNORE) {
      ADEBUG << "Skip ignored obstacle [" << obstacle_id << "]";
      continue;
    } else if (feature.priority().priority() == ObstaclePriority::CAUTION) {
      caution_thread_idx = caution_thread_idx % FLAGS_max_caution_thread_num;
      (*id_obstacle_map)[caution_thread_idx].push_back(obstacle_ptr);
      ADEBUG << "Cautioned obstacle [" << obstacle_id << "] for thread"
             << caution_thread_idx;
      ++caution_thread_idx;
    } else {
      int normal_thread_num =
          FLAGS_max_thread_num - FLAGS_max_caution_thread_num;
      int id_mod =
          obstacle_id % normal_thread_num + FLAGS_max_caution_thread_num;
      (*id_obstacle_map)[id_mod].push_back(obstacle_ptr);
      ADEBUG << "Normal obstacle [" << obstacle_id << "] for thread" << id_mod;
    }
  }
}

}  // namespace

EvaluatorManager::EvaluatorManager() { RegisterEvaluators(); }

void EvaluatorManager::RegisterEvaluators() {
  RegisterEvaluator(ObstacleConf::MLP_EVALUATOR);
  RegisterEvaluator(ObstacleConf::COST_EVALUATOR);
  RegisterEvaluator(ObstacleConf::CRUISE_MLP_EVALUATOR);
  RegisterEvaluator(ObstacleConf::JUNCTION_MLP_EVALUATOR);
  RegisterEvaluator(ObstacleConf::CYCLIST_KEEP_LANE_EVALUATOR);
  RegisterEvaluator(ObstacleConf::LANE_SCANNING_EVALUATOR);
  RegisterEvaluator(ObstacleConf::LANE_AGGREGATING_EVALUATOR);
  RegisterEvaluator(ObstacleConf::PEDESTRIAN_INTERACTION_EVALUATOR);
  RegisterEvaluator(ObstacleConf::JUNCTION_MAP_EVALUATOR);
  RegisterEvaluator(ObstacleConf::SEMANTIC_LSTM_EVALUATOR);
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
      switch (obstacle_conf.obstacle_type()) {
        case PerceptionObstacle::VEHICLE: {
          if (obstacle_conf.obstacle_status() == ObstacleConf::ON_LANE) {
            if (obstacle_conf.priority_type() == ObstaclePriority::CAUTION) {
              vehicle_on_lane_caution_evaluator_ =
                  obstacle_conf.evaluator_type();
            } else {
              vehicle_on_lane_evaluator_ = obstacle_conf.evaluator_type();
            }
          }
          if (obstacle_conf.obstacle_status() == ObstacleConf::IN_JUNCTION) {
            if (obstacle_conf.priority_type() == ObstaclePriority::CAUTION) {
              vehicle_in_junction_caution_evaluator_ =
                  obstacle_conf.evaluator_type();
            } else {
              vehicle_in_junction_evaluator_ = obstacle_conf.evaluator_type();
            }
          }
          break;
        }
        case PerceptionObstacle::BICYCLE: {
          if (obstacle_conf.obstacle_status() == ObstacleConf::ON_LANE) {
            cyclist_on_lane_evaluator_ = obstacle_conf.evaluator_type();
          }
          break;
        }
        case PerceptionObstacle::PEDESTRIAN: {
          if (FLAGS_prediction_offline_mode ==
                  PredictionConstants::kDumpDataForLearning ||
              obstacle_conf.priority_type() == ObstaclePriority::CAUTION) {
            pedestrian_evaluator_ = obstacle_conf.evaluator_type();
            break;
          }
        }
        case PerceptionObstacle::UNKNOWN: {
          if (obstacle_conf.obstacle_status() == ObstacleConf::ON_LANE) {
            default_on_lane_evaluator_ = obstacle_conf.evaluator_type();
          }
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

  if (FLAGS_enable_semantic_map) {
    SemanticMap::Instance()->Init();
    ADEBUG << "Init SemanticMap instance.";
  }
}

Evaluator* EvaluatorManager::GetEvaluator(
    const ObstacleConf::EvaluatorType& type) {
  auto it = evaluators_.find(type);
  return it != evaluators_.end() ? it->second.get() : nullptr;
}

void EvaluatorManager::Run(ObstaclesContainer* obstacles_container) {
  if (FLAGS_enable_semantic_map ||
      FLAGS_prediction_offline_mode == PredictionConstants::kDumpFrameEnv) {
    size_t max_num_frame = 10;
    if (FLAGS_prediction_offline_mode == PredictionConstants::kDumpFrameEnv) {
      max_num_frame = 20;
    }
    BuildObstacleIdHistoryMap(obstacles_container, max_num_frame);
    DumpCurrentFrameEnv(obstacles_container);
    if (FLAGS_prediction_offline_mode == PredictionConstants::kDumpFrameEnv) {
      return;
    }
    SemanticMap::Instance()->RunCurrFrame(obstacle_id_history_map_);
  }

  std::vector<Obstacle*> dynamic_env;

  if (FLAGS_enable_multi_thread) {
    IdObstacleListMap id_obstacle_map;
    GroupObstaclesByObstacleIds(obstacles_container, &id_obstacle_map);
    PredictionThreadPool::ForEach(
        id_obstacle_map.begin(), id_obstacle_map.end(),
        [&](IdObstacleListMap::iterator::value_type& obstacles_iter) {
          for (auto obstacle_ptr : obstacles_iter.second) {
            EvaluateObstacle(obstacle_ptr, obstacles_container, dynamic_env);
          }
        });
  } else {
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
}

void EvaluatorManager::EvaluateObstacle(Obstacle* obstacle,
                                        ObstaclesContainer* obstacles_container,
                                        std::vector<Obstacle*> dynamic_env) {
  Evaluator* evaluator = nullptr;
  // Select different evaluators depending on the obstacle's type.
  switch (obstacle->type()) {
    case PerceptionObstacle::VEHICLE: {
      if (obstacle->IsCaution() && !obstacle->IsSlow()) {
        if (obstacle->IsNearJunction()) {
          evaluator = GetEvaluator(vehicle_in_junction_caution_evaluator_);
        } else if (obstacle->IsOnLane()) {
          evaluator = GetEvaluator(vehicle_on_lane_caution_evaluator_);
        } else {
          evaluator = GetEvaluator(vehicle_default_caution_evaluator_);
        }
        CHECK_NOTNULL(evaluator);
        // Evaluate and break if success
        if (evaluator->Evaluate(obstacle, obstacles_container)) {
          break;
        } else {
          AERROR << "Obstacle: " << obstacle->id()
                 << " caution evaluator failed, downgrade to normal level!";
        }
      }
      // if obstacle is not caution or caution_evaluator run failed
      if (obstacle->HasJunctionFeatureWithExits() &&
          !obstacle->IsCloseToJunctionExit()) {
        evaluator = GetEvaluator(vehicle_in_junction_evaluator_);
      } else if (obstacle->IsOnLane()) {
        evaluator = GetEvaluator(vehicle_on_lane_evaluator_);
      } else {
        ADEBUG << "Obstacle: " << obstacle->id()
               << " is neither on lane, nor in junction. Skip evaluating.";
        break;
      }
      CHECK_NOTNULL(evaluator);
      if (evaluator->GetName() == "LANE_SCANNING_EVALUATOR") {
        evaluator->Evaluate(obstacle, obstacles_container, dynamic_env);
      } else {
        evaluator->Evaluate(obstacle, obstacles_container);
      }
      break;
    }
    case PerceptionObstacle::BICYCLE: {
      if (obstacle->IsOnLane()) {
        evaluator = GetEvaluator(cyclist_on_lane_evaluator_);
        CHECK_NOTNULL(evaluator);
        evaluator->Evaluate(obstacle, obstacles_container);
      }
      break;
    }
    case PerceptionObstacle::PEDESTRIAN: {
      if (FLAGS_prediction_offline_mode ==
              PredictionConstants::kDumpDataForLearning ||
          obstacle->latest_feature().priority().priority() ==
              ObstaclePriority::CAUTION) {
        evaluator = GetEvaluator(pedestrian_evaluator_);
        CHECK_NOTNULL(evaluator);
        evaluator->Evaluate(obstacle, obstacles_container);
        break;
      }
    }
    default: {
      if (obstacle->IsOnLane()) {
        evaluator = GetEvaluator(default_on_lane_evaluator_);
        CHECK_NOTNULL(evaluator);
        evaluator->Evaluate(obstacle, obstacles_container);
      }
      break;
    }
  }
}

void EvaluatorManager::EvaluateObstacle(
    Obstacle* obstacle, ObstaclesContainer* obstacles_container) {
  std::vector<Obstacle*> dummy_dynamic_env;
  EvaluateObstacle(obstacle, obstacles_container, dummy_dynamic_env);
}

void EvaluatorManager::BuildObstacleIdHistoryMap(
    ObstaclesContainer* obstacles_container, size_t max_num_frame) {
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
        std::min(max_num_frame, obstacle->history_size());
    for (size_t i = 0; i < num_frames; ++i) {
      const Feature& obstacle_feature = obstacle->feature(i);
      Feature feature;
      feature.set_id(obstacle_feature.id());
      feature.set_timestamp(obstacle_feature.timestamp());
      feature.set_type(obstacle_feature.type());
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

void EvaluatorManager::DumpCurrentFrameEnv(
    ObstaclesContainer* obstacles_container) {
  FrameEnv curr_frame_env;
  curr_frame_env.set_timestamp(obstacles_container->timestamp());
  for (const auto obstacle_id_history_pair : obstacle_id_history_map_) {
    int id = obstacle_id_history_pair.first;
    if (id != FLAGS_ego_vehicle_id) {
      curr_frame_env.add_obstacles_history()->CopyFrom(
          obstacle_id_history_pair.second);
    } else {
      curr_frame_env.mutable_ego_history()->CopyFrom(
          obstacle_id_history_pair.second);
    }
  }
  FeatureOutput::InsertFrameEnv(curr_frame_env);
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
    case ObstacleConf::LANE_AGGREGATING_EVALUATOR: {
      evaluator_ptr.reset(new LaneAggregatingEvaluator());
      break;
    }
    case ObstacleConf::PEDESTRIAN_INTERACTION_EVALUATOR: {
      evaluator_ptr.reset(new PedestrianInteractionEvaluator());
      break;
    }
    case ObstacleConf::JUNCTION_MAP_EVALUATOR: {
      evaluator_ptr.reset(new JunctionMapEvaluator());
      break;
    }
    case ObstacleConf::SEMANTIC_LSTM_EVALUATOR: {
      evaluator_ptr.reset(new SemanticLSTMEvaluator());
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
