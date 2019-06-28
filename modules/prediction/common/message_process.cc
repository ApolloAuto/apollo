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

#include "modules/prediction/common/message_process.h"

#include <algorithm>
#include <vector>

#include "cyber/common/file.h"
#include "cyber/record/record_reader.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/prediction/common/feature_output.h"
#include "modules/prediction/common/junction_analyzer.h"
#include "modules/prediction/common/prediction_constants.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_system_gflags.h"
#include "modules/prediction/common/validation_checker.h"
#include "modules/prediction/evaluator/evaluator_manager.h"
#include "modules/prediction/predictor/predictor_manager.h"
#include "modules/prediction/proto/offline_features.pb.h"
#include "modules/prediction/scenario/prioritization/obstacles_prioritizer.h"
#include "modules/prediction/scenario/right_of_way/right_of_way.h"
#include "modules/prediction/scenario/scenario_manager.h"
#include "modules/prediction/util/data_extraction.h"

namespace apollo {
namespace prediction {

using apollo::common::adapter::AdapterConfig;
using apollo::localization::LocalizationEstimate;
using apollo::perception::PerceptionObstacle;
using apollo::perception::PerceptionObstacles;
using apollo::planning::ADCTrajectory;
using cyber::record::RecordMessage;
using cyber::record::RecordReader;

bool MessageProcess::Init() {
  // Load prediction conf
  PredictionConf prediction_conf;
  if (!cyber::common::GetProtoFromFile(FLAGS_prediction_conf_file,
                                       &prediction_conf)) {
    AERROR << "Unable to load prediction conf file: "
           << FLAGS_prediction_conf_file;
    return false;
  }
  ADEBUG << "Prediction config file is loaded into: "
         << prediction_conf.ShortDebugString();

  common::adapter::AdapterManagerConfig adapter_conf;
  if (!cyber::common::GetProtoFromFile(FLAGS_prediction_adapter_config_filename,
                                       &adapter_conf)) {
    AERROR << "Unable to load adapter conf file: "
           << FLAGS_prediction_adapter_config_filename;
    return false;
  }
  ADEBUG << "Adapter config file is loaded into: "
         << adapter_conf.ShortDebugString();

  // Initialization of all managers
  ContainerManager::Instance()->Init(adapter_conf);
  EvaluatorManager::Instance()->Init(prediction_conf);
  PredictorManager::Instance()->Init(prediction_conf);

  if (!FLAGS_use_navigation_mode && !PredictionMap::Ready()) {
    AERROR << "Map cannot be loaded.";
    return false;
  }

  return true;
}

void MessageProcess::OnPerception(
    const perception::PerceptionObstacles& perception_obstacles,
    PredictionObstacles* const prediction_obstacles) {
  ADEBUG << "Received a perception message ["
         << perception_obstacles.ShortDebugString() << "].";

  // Get obstacles_container
  auto ptr_obstacles_container =
      ContainerManager::Instance()->GetContainer<ObstaclesContainer>(
          AdapterConfig::PERCEPTION_OBSTACLES);
  CHECK_NOTNULL(ptr_obstacles_container);

  // Get pose_container
  auto ptr_ego_pose_container =
      ContainerManager::Instance()->GetContainer<PoseContainer>(
          AdapterConfig::LOCALIZATION);
  CHECK_NOTNULL(ptr_ego_pose_container);

  // Get adc_trajectory_container
  auto ptr_ego_trajectory_container =
      ContainerManager::Instance()->GetContainer<ADCTrajectoryContainer>(
          AdapterConfig::PLANNING_TRAJECTORY);
  CHECK_NOTNULL(ptr_ego_trajectory_container);

  // Insert ADC into the obstacle_container.
  const PerceptionObstacle* ptr_ego_vehicle =
      ptr_ego_pose_container->ToPerceptionObstacle();
  if (ptr_ego_vehicle != nullptr) {
    ptr_obstacles_container->InsertPerceptionObstacle(
        *ptr_ego_vehicle, ptr_ego_vehicle->timestamp());
    double x = ptr_ego_vehicle->position().x();
    double y = ptr_ego_vehicle->position().y();
    ADEBUG << "Get ADC position [" << std::fixed << std::setprecision(6) << x
           << ", " << std::fixed << std::setprecision(6) << y << "].";
    ptr_ego_trajectory_container->SetPosition({x, y});
  }

  // Insert perception_obstacles
  ptr_obstacles_container->Insert(perception_obstacles);

  // Ignore some obstacles
  ObstaclesPrioritizer::Instance()->AssignIgnoreLevel();

  // Scenario analysis
  ScenarioManager::Instance()->Run();

  // Build junction feature for the obstacles in junction
  const Scenario& scenario = ScenarioManager::Instance()->scenario();
  if (scenario.type() == Scenario::JUNCTION && scenario.has_junction_id()) {
    JunctionAnalyzer::Init(scenario.junction_id());
    ptr_obstacles_container->BuildJunctionFeature();
  }

  // Build lane graph
  ptr_obstacles_container->BuildLaneGraph();

  // Assign CautionLevel for obstacles
  ObstaclesPrioritizer::Instance()->AssignCautionLevel();

  // Analyze RightOfWay for the caution obstacles
  RightOfWay::Analyze();

  // Insert features to FeatureOutput for offline_mode
  if (FLAGS_prediction_offline_mode == PredictionConstants::kDumpFeatureProto) {
    for (const int id :
         ptr_obstacles_container->curr_frame_movable_obstacle_ids()) {
      Obstacle* obstacle_ptr = ptr_obstacles_container->GetObstacle(id);
      if (obstacle_ptr == nullptr) {
        AERROR << "Null obstacle found.";
        continue;
      }
      if (!obstacle_ptr->latest_feature().IsInitialized()) {
        AERROR << "Obstacle [" << id << "] has no latest feature.";
        continue;
      }
      // TODO(all): the adc trajectory should be part of features for learning
      //            algorithms rather than part of the feature.proto
      /*
      *obstacle_ptr->mutable_latest_feature()->mutable_adc_trajectory_point() =
          ptr_ego_trajectory_container->adc_trajectory().trajectory_point();
      */
      FeatureOutput::InsertFeatureProto(obstacle_ptr->latest_feature());
      ADEBUG << "Insert feature into feature output";
    }
    // Not doing evaluation on offline mode
    return;
  }

  // Make evaluations
  EvaluatorManager::Instance()->Run();
  if (FLAGS_prediction_offline_mode ==
          PredictionConstants::kDumpDataForLearning ||
      FLAGS_prediction_offline_mode == PredictionConstants::kDumpFrameEnv) {
    return;
  }
  // Make predictions
  PredictorManager::Instance()->Run();

  // Get predicted obstacles
  *prediction_obstacles = PredictorManager::Instance()->prediction_obstacles();
}

void MessageProcess::OnLocalization(
    const localization::LocalizationEstimate& localization) {
  auto ptr_ego_pose_container =
      ContainerManager::Instance()->GetContainer<PoseContainer>(
          AdapterConfig::LOCALIZATION);
  CHECK(ptr_ego_pose_container != nullptr);
  ptr_ego_pose_container->Insert(localization);

  ADEBUG << "Received a localization message ["
         << localization.ShortDebugString() << "].";
}

void MessageProcess::OnPlanning(const planning::ADCTrajectory& adc_trajectory) {
  auto ptr_ego_trajectory_container =
      ContainerManager::Instance()->GetContainer<ADCTrajectoryContainer>(
          AdapterConfig::PLANNING_TRAJECTORY);
  CHECK(ptr_ego_trajectory_container != nullptr);
  ptr_ego_trajectory_container->Insert(adc_trajectory);

  ADEBUG << "Received a planning message [" << adc_trajectory.ShortDebugString()
         << "].";
}

void MessageProcess::ProcessOfflineData(const std::string& record_filename) {
  RecordReader reader(record_filename);
  RecordMessage message;
  while (reader.ReadMessage(&message)) {
    if (message.channel_name == FLAGS_perception_obstacle_topic) {
      PerceptionObstacles perception_obstacles;
      if (perception_obstacles.ParseFromString(message.content)) {
        PredictionObstacles prediction_obstacles;
        OnPerception(perception_obstacles, &prediction_obstacles);
      }
    } else if (message.channel_name == FLAGS_localization_topic) {
      LocalizationEstimate localization;
      if (localization.ParseFromString(message.content)) {
        OnLocalization(localization);
      }
    } else if (message.channel_name == FLAGS_planning_trajectory_topic) {
      ADCTrajectory adc_trajectory;
      if (adc_trajectory.ParseFromString(message.content)) {
        OnPlanning(adc_trajectory);
      }
    }
  }
}

}  // namespace prediction
}  // namespace apollo
