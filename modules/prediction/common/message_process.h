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

#include <string>

#include "modules/localization/proto/localization.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/storytelling/proto/story.pb.h"

namespace apollo {
namespace prediction {

class MessageProcess {
 public:
  MessageProcess() = delete;

  static bool Init();

  static bool InitContainers();

  static bool InitEvaluators();

  static bool InitPredictors();

  static void ContainerProcess(
      const perception::PerceptionObstacles &perception_obstacles);

  static void OnPerception(
      const perception::PerceptionObstacles &perception_obstacles,
      PredictionObstacles *const prediction_obstacles);

  static void OnLocalization(
      const localization::LocalizationEstimate &localization);

  static void OnPlanning(const planning::ADCTrajectory &adc_trajectory);

  static void OnStoryTelling(const storytelling::Stories &story);

  static void ProcessOfflineData(const std::string &record_filename);
};

}  // namespace prediction
}  // namespace apollo
