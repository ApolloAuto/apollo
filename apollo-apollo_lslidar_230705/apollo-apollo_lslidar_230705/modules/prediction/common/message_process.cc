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

#include <memory>

#include "cyber/common/file.h"
#include "cyber/record/record_reader.h"
#include "cyber/record/record_writer.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/prediction/common/feature_output.h"
#include "modules/prediction/common/junction_analyzer.h"
#include "modules/prediction/common/prediction_constants.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_system_gflags.h"
#include "modules/prediction/common/validation_checker.h"
#include "modules/prediction/container/storytelling/storytelling_container.h"
#include "modules/prediction/evaluator/evaluator_manager.h"
#include "modules/prediction/predictor/predictor_manager.h"
#include "modules/prediction/proto/offline_features.pb.h"
#include "modules/prediction/scenario/interaction_filter/interaction_filter.h"
#include "modules/prediction/scenario/prioritization/obstacles_prioritizer.h"
#include "modules/prediction/scenario/right_of_way/right_of_way.h"
#include "modules/common/util/data_extraction.h"

namespace apollo {
namespace prediction {

using apollo::common::adapter::AdapterConfig;
using apollo::cyber::proto::SingleMessage;
using apollo::cyber::record::RecordMessage;
using apollo::cyber::record::RecordReader;
using apollo::cyber::record::RecordWriter;
using apollo::localization::LocalizationEstimate;
using apollo::perception::PerceptionObstacle;
using apollo::perception::PerceptionObstacles;
using apollo::planning::ADCTrajectory;
using apollo::storytelling::Stories;

bool MessageProcess::Init(ContainerManager* container_manager,
                          EvaluatorManager* evaluator_manager,
                          PredictorManager* predictor_manager,
                          const PredictionConf& prediction_conf) {
  InitContainers(container_manager);
  InitEvaluators(evaluator_manager, prediction_conf);
  InitPredictors(predictor_manager, prediction_conf);

  if (!FLAGS_use_navigation_mode && !PredictionMap::Ready()) {
    AERROR << "Map cannot be loaded.";
    return false;
  }

  return true;
}

bool MessageProcess::InitContainers(ContainerManager* container_manager) {
  common::adapter::AdapterManagerConfig adapter_conf;
  if (!cyber::common::GetProtoFromFile(FLAGS_prediction_adapter_config_filename,
                                       &adapter_conf)) {
    AERROR << "Unable to load adapter conf file: "
           << FLAGS_prediction_adapter_config_filename;
    return false;
  }
  ADEBUG << "Adapter config file is loaded into: "
         << adapter_conf.ShortDebugString();

  container_manager->Init(adapter_conf);
  return true;
}

bool MessageProcess::InitEvaluators(EvaluatorManager* evaluator_manager,
                                    const PredictionConf& prediction_conf) {
  evaluator_manager->Init(prediction_conf);
  return true;
}

bool MessageProcess::InitPredictors(PredictorManager* predictor_manager,
                                    const PredictionConf& prediction_conf) {
  predictor_manager->Init(prediction_conf);
  return true;
}

void MessageProcess::ContainerProcess(
    const std::shared_ptr<ContainerManager>& container_manager,
    const perception::PerceptionObstacles& perception_obstacles,
    ScenarioManager* scenario_manager) {
  ADEBUG << "Received a perception message ["
         << perception_obstacles.ShortDebugString() << "].";

  // Get obstacles_container
  auto ptr_obstacles_container =
      container_manager->GetContainer<ObstaclesContainer>(
          AdapterConfig::PERCEPTION_OBSTACLES);
  CHECK_NOTNULL(ptr_obstacles_container);
  ptr_obstacles_container->CleanUp();

  // Get pose_container
  auto ptr_ego_pose_container = container_manager->GetContainer<PoseContainer>(
      AdapterConfig::LOCALIZATION);
  CHECK_NOTNULL(ptr_ego_pose_container);

  // Get adc_trajectory_container
  auto ptr_ego_trajectory_container =
      container_manager->GetContainer<ADCTrajectoryContainer>(
          AdapterConfig::PLANNING_TRAJECTORY);
  CHECK_NOTNULL(ptr_ego_trajectory_container);

  // Get storytelling_container
  auto ptr_storytelling_container =
      container_manager->GetContainer<StoryTellingContainer>(
          AdapterConfig::STORYTELLING);
  CHECK_NOTNULL(ptr_storytelling_container);

  // Insert ADC into the obstacle_container.
  const PerceptionObstacle* ptr_ego_vehicle =
      ptr_ego_pose_container->ToPerceptionObstacle();
  if (ptr_ego_vehicle != nullptr) {
    double perception_obs_timestamp = ptr_ego_vehicle->timestamp();
    if (perception_obstacles.has_header() &&
        perception_obstacles.header().has_timestamp_sec()) {
      ADEBUG << "Correcting " << std::fixed << std::setprecision(6)
             << ptr_ego_vehicle->timestamp() << " to " << std::fixed
             << std::setprecision(6)
             << perception_obstacles.header().timestamp_sec();
      perception_obs_timestamp = perception_obstacles.header().timestamp_sec();
    }
    ptr_obstacles_container->InsertPerceptionObstacle(*ptr_ego_vehicle,
                                                      perception_obs_timestamp);
    double x = ptr_ego_vehicle->position().x();
    double y = ptr_ego_vehicle->position().y();
    ADEBUG << "Get ADC position [" << std::fixed << std::setprecision(6) << x
           << ", " << std::fixed << std::setprecision(6) << y << "].";
    ptr_ego_trajectory_container->SetPosition({x, y});
  }

  // Insert perception_obstacles
  ptr_obstacles_container->Insert(perception_obstacles);

  ObstaclesPrioritizer obstacles_prioritizer(container_manager);

  InteractionFilter interaction_filter(container_manager);

  // Ignore some obstacles
  obstacles_prioritizer.AssignIgnoreLevel();

  // Scenario analysis
  scenario_manager->Run(container_manager.get());

  // Build junction feature for the obstacles in junction
  const Scenario scenario = scenario_manager->scenario();
  if (scenario.type() == Scenario::JUNCTION && scenario.has_junction_id()) {
    ptr_obstacles_container->GetJunctionAnalyzer()->Init(
        scenario.junction_id());
    ptr_obstacles_container->BuildJunctionFeature();
  }

  // Build lane graph
  ptr_obstacles_container->BuildLaneGraph();

  // Assign CautionLevel for obstacles
  obstacles_prioritizer.AssignCautionLevel();

  // Add interactive tag
  if (FLAGS_enable_interactive_tag) {
    interaction_filter.AssignInteractiveTag();
  }

  // Analyze RightOfWay for the caution obstacles
  RightOfWay::Analyze(container_manager.get());
}

void MessageProcess::OnPerception(
    const perception::PerceptionObstacles& perception_obstacles,
    const std::shared_ptr<ContainerManager>& container_manager,
    EvaluatorManager* evaluator_manager, PredictorManager* predictor_manager,
    ScenarioManager* scenario_manager,
    PredictionObstacles* const prediction_obstacles) {
  ContainerProcess(container_manager, perception_obstacles, scenario_manager);

  auto ptr_obstacles_container =
      container_manager->GetContainer<ObstaclesContainer>(
          AdapterConfig::PERCEPTION_OBSTACLES);
  CHECK_NOTNULL(ptr_obstacles_container);

  auto ptr_ego_trajectory_container =
      container_manager->GetContainer<ADCTrajectoryContainer>(
          AdapterConfig::PLANNING_TRAJECTORY);
  CHECK_NOTNULL(ptr_ego_trajectory_container);

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
      *obstacle_ptr->mutable_latest_feature()->mutable_adc_trajectory_point() =
          ptr_ego_trajectory_container->adc_trajectory().trajectory_point();

      // adc trajectory timestamp
      obstacle_ptr->mutable_latest_feature()->set_adc_timestamp(
          ptr_ego_trajectory_container->adc_trajectory()
          .header().timestamp_sec());

      // ego pose_container
      auto ptr_ego_pose = container_manager->GetContainer<PoseContainer>(
          AdapterConfig::LOCALIZATION);
      CHECK_NOTNULL(ptr_ego_pose);

      // adc localization
      obstacle_ptr->mutable_latest_feature()->mutable_adc_localization()->
        CopyFrom(*ptr_ego_pose->ToPerceptionObstacle());

      FeatureOutput::InsertFeatureProto(obstacle_ptr->latest_feature());
      ADEBUG << "Insert feature into feature output";
    }
    // Not doing evaluation on offline mode
    return;
  }

  // Make evaluations
  evaluator_manager->Run(ptr_ego_trajectory_container,
                         ptr_obstacles_container);
  if (FLAGS_prediction_offline_mode ==
          PredictionConstants::kDumpDataForLearning ||
      FLAGS_prediction_offline_mode == PredictionConstants::kDumpFrameEnv) {
    return;
  }
  // Make predictions
  predictor_manager->Run(perception_obstacles, ptr_ego_trajectory_container,
                         ptr_obstacles_container);

  // Get predicted obstacles
  *prediction_obstacles = predictor_manager->prediction_obstacles();
}

void MessageProcess::OnLocalization(
    ContainerManager* container_manager,
    const localization::LocalizationEstimate& localization) {
  auto ptr_ego_pose_container = container_manager->GetContainer<PoseContainer>(
      AdapterConfig::LOCALIZATION);
  ACHECK(ptr_ego_pose_container != nullptr);
  ptr_ego_pose_container->Insert(localization);

  ADEBUG << "Received a localization message ["
         << localization.ShortDebugString() << "].";
}

void MessageProcess::OnPlanning(ContainerManager* container_manager,
                                const planning::ADCTrajectory& adc_trajectory) {
  auto ptr_ego_trajectory_container =
      container_manager->GetContainer<ADCTrajectoryContainer>(
          AdapterConfig::PLANNING_TRAJECTORY);
  ACHECK(ptr_ego_trajectory_container != nullptr);
  ptr_ego_trajectory_container->Insert(adc_trajectory);

  ADEBUG << "Received a planning message [" << adc_trajectory.ShortDebugString()
         << "].";

  auto ptr_storytelling_container =
      container_manager->GetContainer<StoryTellingContainer>(
          AdapterConfig::STORYTELLING);
  CHECK_NOTNULL(ptr_storytelling_container);
  ptr_ego_trajectory_container->SetJunction(
      ptr_storytelling_container->ADCJunctionId(),
      ptr_storytelling_container->ADCDistanceToJunction());
}

void MessageProcess::OnStoryTelling(ContainerManager* container_manager,
                                    const Stories& story) {
  auto ptr_storytelling_container =
      container_manager->GetContainer<StoryTellingContainer>(
          AdapterConfig::STORYTELLING);
  CHECK_NOTNULL(ptr_storytelling_container);
  ptr_storytelling_container->Insert(story);

  ADEBUG << "Received a storytelling message [" << story.ShortDebugString()
         << "].";
}

void MessageProcess::ProcessOfflineData(
    const PredictionConf& prediction_conf,
    const std::shared_ptr<ContainerManager>& container_manager,
    EvaluatorManager* evaluator_manager, PredictorManager* predictor_manager,
    ScenarioManager* scenario_manager, const std::string& record_filepath) {
  RecordReader reader(record_filepath);
  RecordMessage message;
  RecordWriter writer;
  if (FLAGS_prediction_offline_mode == PredictionConstants::kDumpRecord) {
    writer.Open(record_filepath + ".new_prediction");
  }
  while (reader.ReadMessage(&message)) {
    if (message.channel_name ==
        prediction_conf.topic_conf().perception_obstacle_topic()) {
      PerceptionObstacles perception_obstacles;
      if (perception_obstacles.ParseFromString(message.content)) {
        if (FLAGS_prediction_offline_mode == PredictionConstants::kDumpRecord) {
          writer.WriteMessage<PerceptionObstacles>(
              message.channel_name, perception_obstacles, message.time);
        }
        PredictionObstacles prediction_obstacles;
        OnPerception(perception_obstacles, container_manager, evaluator_manager,
                     predictor_manager, scenario_manager,
                     &prediction_obstacles);
        if (FLAGS_prediction_offline_mode == PredictionConstants::kDumpRecord) {
          writer.WriteMessage<PredictionObstacles>(
              prediction_conf.topic_conf().perception_obstacle_topic(),
              prediction_obstacles, message.time);
          AINFO << "Generated a new prediction message.";
        }
      }
    } else if (message.channel_name ==
               prediction_conf.topic_conf().localization_topic()) {
      LocalizationEstimate localization;
      if (localization.ParseFromString(message.content)) {
        if (FLAGS_prediction_offline_mode == PredictionConstants::kDumpRecord) {
          writer.WriteMessage<LocalizationEstimate>(message.channel_name,
                                                    localization, message.time);
        }
        OnLocalization(container_manager.get(), localization);
      }
    } else if (message.channel_name ==
               prediction_conf.topic_conf().planning_trajectory_topic()) {
      ADCTrajectory adc_trajectory;
      if (adc_trajectory.ParseFromString(message.content)) {
        OnPlanning(container_manager.get(), adc_trajectory);
      }
    }
  }
  if (FLAGS_prediction_offline_mode == PredictionConstants::kDumpRecord) {
    writer.Close();
  }
}

}  // namespace prediction
}  // namespace apollo
