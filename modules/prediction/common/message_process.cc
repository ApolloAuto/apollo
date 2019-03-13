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
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_system_gflags.h"
#include "modules/prediction/common/validation_checker.h"
#include "modules/prediction/evaluator/evaluator_manager.h"
#include "modules/prediction/predictor/predictor_manager.h"
#include "modules/prediction/proto/offline_features.pb.h"
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
  // Insert obstacle
  auto end_time1 = std::chrono::system_clock::now();
  auto ptr_obstacles_container =
      ContainerManager::Instance()->GetContainer<ObstaclesContainer>(
          AdapterConfig::PERCEPTION_OBSTACLES);
  CHECK(ptr_obstacles_container != nullptr);

  // Insert ADC into the obstacle_container.
  auto ptr_ego_pose_container =
      ContainerManager::Instance()->GetContainer<PoseContainer>(
          AdapterConfig::LOCALIZATION);
  auto ptr_ego_trajectory_container =
      ContainerManager::Instance()->GetContainer<ADCTrajectoryContainer>(
          AdapterConfig::PLANNING_TRAJECTORY);
  CHECK(ptr_ego_pose_container != nullptr &&
        ptr_ego_trajectory_container != nullptr);
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
  auto end_time2 = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_time2 - end_time1;
  ADEBUG << "Time to insert ADC: " << diff.count() * 1000 << " msec.";

  // Insert perception_obstacles
  ptr_obstacles_container->Insert(perception_obstacles);
  auto end_time3 = std::chrono::system_clock::now();
  diff = end_time3 - end_time2;
  ADEBUG << "Time to insert obstacles: " << diff.count() * 1000 << " msec.";

  // Scenario analysis
  ScenarioManager::Instance()->Run();
  auto end_time4 = std::chrono::system_clock::now();
  diff = end_time4 - end_time3;
  ADEBUG << "Time for scenario_manager: " << diff.count() * 1000 << " msec.";

  // If in junction, BuildJunctionFeature();
  // If not, BuildLaneGraph().
  const Scenario& scenario = ScenarioManager::Instance()->scenario();
  if (scenario.type() == Scenario::JUNCTION && scenario.has_junction_id() &&
      FLAGS_enable_junction_feature) {
    JunctionAnalyzer::Init(scenario.junction_id());
    ptr_obstacles_container->BuildJunctionFeature();
  }
  auto end_time5 = std::chrono::system_clock::now();
  diff = end_time5 - end_time4;
  ADEBUG << "Time to build junction features: " << diff.count() * 1000
         << " msec.";
  ptr_obstacles_container->BuildLaneGraph();
  auto end_time6 = std::chrono::system_clock::now();
  diff = end_time6 - end_time5;
  ADEBUG << "Time to build cruise features: " << diff.count() * 1000
         << " msec.";
  ADEBUG << "Received a perception message ["
         << perception_obstacles.ShortDebugString() << "].";

  // Insert features to FeatureOutput for offline_mode
  if (FLAGS_prediction_offline_mode == 1) {
    for (const int id :
         ptr_obstacles_container->curr_frame_predictable_obstacle_ids()) {
      Obstacle* obstacle_ptr = ptr_obstacles_container->GetObstacle(id);
      if (obstacle_ptr == nullptr) {
        AERROR << "Null obstacle found.";
        continue;
      } else if (!obstacle_ptr->latest_feature().IsInitialized()) {
        AERROR << "Obstacle [" << id << "] has no latest feature.";
        return;
      }
      FeatureOutput::InsertFeatureProto(obstacle_ptr->latest_feature());
      ADEBUG << "Insert feature into feature output";
    }
    // Not doing evaluation on offline mode
    return;
  }

  // Make evaluations
  EvaluatorManager::Instance()->Run();
  auto end_time7 = std::chrono::system_clock::now();
  diff = end_time7 - end_time6;
  ADEBUG << "Time to evaluate: " << diff.count() * 1000 << " msec.";

  // Make predictions
  PredictorManager::Instance()->Run();
  auto end_time8 = std::chrono::system_clock::now();
  diff = end_time8 - end_time7;
  ADEBUG << "Time to predict: " << diff.count() * 1000 << " msec.";

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
