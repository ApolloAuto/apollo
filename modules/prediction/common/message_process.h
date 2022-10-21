/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

/**
 * @file
 */

#pragma once

#include <memory>
#include <string>

#include "cyber/proto/record.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/evaluator/evaluator_manager.h"
#include "modules/prediction/predictor/predictor_manager.h"
#include "modules/prediction/proto/prediction_conf.pb.h"
#include "modules/common_msgs/prediction_msgs/prediction_obstacle.pb.h"
#include "modules/prediction/scenario/scenario_manager.h"
#include "modules/common_msgs/storytelling_msgs/story.pb.h"

namespace apollo {
namespace prediction {

class MessageProcess {
 public:
  MessageProcess() = delete;

  static bool Init(ContainerManager *container_manager,
                   EvaluatorManager *evaluator_manager,
                   PredictorManager *predictor_manager,
                   const PredictionConf &prediction_conf);

  static bool InitContainers(ContainerManager *container_manager);

  static bool InitEvaluators(EvaluatorManager *evaluator_manager,
                             const PredictionConf &prediction_conf);

  static bool InitPredictors(PredictorManager *predictor_manager,
                             const PredictionConf &prediction_conf);

  static void ContainerProcess(
      const std::shared_ptr<ContainerManager> &container_manager,
      const perception::PerceptionObstacles &perception_obstacles,
      ScenarioManager *scenario_manger);

  static void OnPerception(
      const perception::PerceptionObstacles &perception_obstacles,
      const std::shared_ptr<ContainerManager> &container_manager,
      EvaluatorManager *evaluator_manager, PredictorManager *predictor_manager,
      ScenarioManager *scenario_manager,
      PredictionObstacles *const prediction_obstacles);

  static void OnLocalization(
      ContainerManager *container_manager,
      const localization::LocalizationEstimate &localization);

  static void OnPlanning(ContainerManager *container_manager,
                         const planning::ADCTrajectory &adc_trajectory);

  static void OnStoryTelling(ContainerManager *container_manager,
                             const storytelling::Stories &story);

  static void ProcessOfflineData(
      const PredictionConf &prediction_conf,
      const std::shared_ptr<ContainerManager> &container_manager,
      EvaluatorManager *evaluator_manager, PredictorManager *predictor_manager,
      ScenarioManager *scenario_manager, const std::string &record_filepath);
};

}  // namespace prediction
}  // namespace apollo
