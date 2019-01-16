/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/prediction/prediction_component.h"

#include <algorithm>
#include <vector>

#include "cyber/common/file.h"
#include "cyber/record/record_reader.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"

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
using apollo::common::time::Clock;
using apollo::localization::LocalizationEstimate;
using apollo::perception::PerceptionObstacle;
using apollo::perception::PerceptionObstacles;
using apollo::planning::ADCTrajectory;
using cyber::record::RecordMessage;
using cyber::record::RecordReader;

PredictionComponent::~PredictionComponent() {}

std::string PredictionComponent::Name() const {
  return FLAGS_prediction_module_name;
}

void PredictionComponent::ProcessOfflineData(const std::string& filename) {
  RecordReader reader(filename);
  RecordMessage message;
  while (reader.ReadMessage(&message)) {
    if (message.channel_name == FLAGS_perception_obstacle_topic) {
      PerceptionObstacles perception_obstacles;
      if (perception_obstacles.ParseFromString(message.content)) {
        OnPerception(perception_obstacles);
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

void PredictionComponent::OfflineProcessFeatureProtoFile(
    const std::string& features_proto_file_name) {
  auto obstacles_container_ptr = ContainerManager::Instance()->GetContainer<
      ObstaclesContainer>(AdapterConfig::PERCEPTION_OBSTACLES);
  obstacles_container_ptr->Clear();
  Features features;
  apollo::cyber::common::GetProtoFromBinaryFile(
      features_proto_file_name, &features);
  for (const Feature& feature : features.feature()) {
    obstacles_container_ptr->InsertFeatureProto(feature);
    Obstacle* obstacle_ptr = obstacles_container_ptr->GetObstacle(feature.id());
    EvaluatorManager::Instance()->EvaluateObstacle(obstacle_ptr);
  }
}

bool PredictionComponent::Init() {
  component_start_time_ = Clock::NowInSeconds();

  // Load prediction conf
  PredictionConf prediction_conf;
  if (!common::util::GetProtoFromFile(FLAGS_prediction_conf_file,
                                      &prediction_conf)) {
    AERROR << "Unable to load prediction conf file: "
           << FLAGS_prediction_conf_file;
    return false;
  }
  ADEBUG << "Prediction config file is loaded into: "
            << prediction_conf.ShortDebugString();

  common::adapter::AdapterManagerConfig adapter_conf;
  if (!common::util::GetProtoFromFile(FLAGS_prediction_adapter_config_filename,
                                      &adapter_conf)) {
    AERROR << "Unable to load adapter conf file: "
           << FLAGS_prediction_adapter_config_filename;
    return false;
  }
  ADEBUG << "Adapter config file is loaded into: "
            << adapter_conf.ShortDebugString();

  planning_reader_ = node_->CreateReader<ADCTrajectory>(
      FLAGS_planning_trajectory_topic, nullptr);

  localization_reader_ =
      node_->CreateReader<localization::LocalizationEstimate>(
          FLAGS_localization_topic, nullptr);

  // Initialization of all managers
  ContainerManager::Instance()->Init(adapter_conf);
  EvaluatorManager::Instance()->Init(prediction_conf);
  PredictorManager::Instance()->Init(prediction_conf);

  if (!FLAGS_use_navigation_mode && !PredictionMap::Ready()) {
    AERROR << "Map cannot be loaded.";
    return false;
  }

  prediction_writer_ =
      node_->CreateWriter<PredictionObstacles>(FLAGS_prediction_topic);

  if (FLAGS_prediction_offline_mode) {
    if (!FeatureOutput::Ready()) {
      AERROR << "Feature output is not ready.";
      return false;
    }
    if (FLAGS_prediction_offline_bags.empty()) {
      return true;  // use listen to ROS topic mode
    }
    std::vector<std::string> inputs;
    common::util::Split(FLAGS_prediction_offline_bags, ':', &inputs);
    for (const auto& input : inputs) {
      std::vector<std::string> offline_bags;
      GetRecordFileNames(boost::filesystem::path(input), &offline_bags);
      std::sort(offline_bags.begin(), offline_bags.end());
      AINFO << "For input " << input << ", found " << offline_bags.size()
            << "  rosbags to process";
      for (std::size_t i = 0; i < offline_bags.size(); ++i) {
        AINFO << "\tProcessing: [ " << i << " / " << offline_bags.size()
              << " ]: " << offline_bags[i];
        ProcessOfflineData(offline_bags[i]);
      }
    }
    FeatureOutput::Close();
    return false;
  }
  return true;
}

void PredictionComponent::OnLocalization(
    const LocalizationEstimate& localization_msg) {
  auto ptr_ego_pose_container =
      ContainerManager::Instance()->GetContainer<PoseContainer>(
          AdapterConfig::LOCALIZATION);
  CHECK(ptr_ego_pose_container != nullptr);
  ptr_ego_pose_container->Insert(localization_msg);

  ADEBUG << "Received a localization message ["
         << localization_msg.ShortDebugString() << "].";
}

void PredictionComponent::OnPlanning(
    const planning::ADCTrajectory& planning_msg) {
  auto ptr_ego_trajectory_container =
      ContainerManager::Instance()->GetContainer<ADCTrajectoryContainer>(
          AdapterConfig::PLANNING_TRAJECTORY);
  CHECK(ptr_ego_trajectory_container != nullptr);
  ptr_ego_trajectory_container->Insert(planning_msg);

  ADEBUG << "Received a planning message ["
         << planning_msg.ShortDebugString() << "].";
}

void PredictionComponent::OnPerception(
    const PerceptionObstacles& perception_msg) {
  // Insert obstacle
  auto end_time1 = std::chrono::system_clock::now();
  auto ptr_obstacles_container = ContainerManager::Instance()->GetContainer<
      ObstaclesContainer>(AdapterConfig::PERCEPTION_OBSTACLES);
  CHECK(ptr_obstacles_container != nullptr);

  ptr_obstacles_container->Insert(perception_msg);
  auto end_time2 = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_time2 - end_time1;
  ADEBUG << "Time to insert obstacles: "
         << diff.count() * 1000 << " msec.";

  // Scenario analysis
  ScenarioManager::Instance()->Run();
  auto end_time3 = std::chrono::system_clock::now();
  diff = end_time3 - end_time2;
  ADEBUG << "Time for scenario_manager: "
         << diff.count() * 1000 << " msec.";

  // If in junction, BuildJunctionFeature();
  // If not, BuildLaneGraph().
  const Scenario& scenario = ScenarioManager::Instance()->scenario();
  if (scenario.type() == Scenario::JUNCTION && scenario.has_junction_id() &&
      FLAGS_enable_junction_feature) {
    JunctionAnalyzer::Init(scenario.junction_id());
    ptr_obstacles_container->BuildJunctionFeature();
  }
  auto end_time4 = std::chrono::system_clock::now();
  diff = end_time4 - end_time3;
  ADEBUG << "Time to build junction features: "
         << diff.count() * 1000 << " msec.";
  ptr_obstacles_container->BuildLaneGraph();
  auto end_time5 = std::chrono::system_clock::now();
  diff = end_time5 - end_time4;
  ADEBUG << "Time to build cruise features: "
         << diff.count() * 1000 << " msec.";
  ADEBUG << "Received a perception message ["
         << perception_msg.ShortDebugString() << "].";

  // Insert ADC into the obstacle_container as well.
  auto ptr_ego_pose_container = ContainerManager::Instance()->GetContainer<
      PoseContainer>(AdapterConfig::LOCALIZATION);
  auto ptr_ego_trajectory_container =
      ContainerManager::Instance()->GetContainer<ADCTrajectoryContainer>(
          AdapterConfig::PLANNING_TRAJECTORY);
  CHECK(ptr_ego_pose_container != nullptr &&
      ptr_ego_trajectory_container != nullptr);
  const PerceptionObstacle* ptr_ego_vehicle =
      ptr_ego_pose_container->ToPerceptionObstacle();
  if (ptr_ego_vehicle != nullptr) {
    ptr_obstacles_container->InsertPerceptionObstacle(*ptr_ego_vehicle,
        ptr_ego_vehicle->timestamp());
    double x = ptr_ego_vehicle->position().x();
    double y = ptr_ego_vehicle->position().y();
    ADEBUG << "Get ADC position [" << std::fixed << std::setprecision(6) << x
              << ", " << std::fixed << std::setprecision(6) << y << "].";
    ptr_ego_trajectory_container->SetPosition({x, y});
  }
  auto end_time6 = std::chrono::system_clock::now();

  // Insert features to FeatureOutput for offline_mode
  if (FLAGS_prediction_offline_mode) {
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
      FeatureOutput::Insert(obstacle_ptr->latest_feature());
      ADEBUG << "Insert feature into feature output";
    }
    // Not doing evaluation on offline mode
    return;
  }

  // Make evaluations
  EvaluatorManager::Instance()->Run();
  auto end_time7 = std::chrono::system_clock::now();
  diff = end_time7 - end_time6;
  ADEBUG << "Time to evaluate: "
        << diff.count() * 1000 << " msec.";

  // Make predictions
  PredictorManager::Instance()->Run();
  auto end_time8 = std::chrono::system_clock::now();
  diff = end_time8 - end_time7;
  ADEBUG << "Time to predict: "
        << diff.count() * 1000 << " msec.";

  // Get predicted obstacles
  auto prediction_obstacles =
      PredictorManager::Instance()->prediction_obstacles();
  prediction_obstacles.set_start_timestamp(frame_start_time_);
  prediction_obstacles.set_end_timestamp(Clock::NowInSeconds());
  prediction_obstacles.mutable_header()->set_lidar_timestamp(
      perception_msg.header().lidar_timestamp());
  prediction_obstacles.mutable_header()->set_camera_timestamp(
      perception_msg.header().camera_timestamp());
  prediction_obstacles.mutable_header()->set_radar_timestamp(
      perception_msg.header().radar_timestamp());

  prediction_obstacles.set_perception_error_code(perception_msg.error_code());

  if (FLAGS_prediction_test_mode) {
    for (auto const& prediction_obstacle :
         prediction_obstacles.prediction_obstacle()) {
      for (auto const& trajectory : prediction_obstacle.trajectory()) {
        for (auto const& trajectory_point : trajectory.trajectory_point()) {
          if (!ValidationChecker::ValidTrajectoryPoint(trajectory_point)) {
            AERROR << "Invalid trajectory point ["
                   << trajectory_point.ShortDebugString() << "]";
            return;
          }
        }
      }
    }
  }

  // Publish output
  common::util::FillHeader(node_->Name(), &prediction_obstacles);
  prediction_writer_->Write(
      std::make_shared<PredictionObstacles>(prediction_obstacles));
}

bool PredictionComponent::Proc(
    const std::shared_ptr<PerceptionObstacles>& perception_obstacles) {
  if (FLAGS_prediction_test_mode &&
      (Clock::NowInSeconds() - component_start_time_ >
       FLAGS_prediction_test_duration)) {
    ADEBUG << "Prediction finished running in test mode";
  }

  // Update relative map if needed
  // AdapterManager::Observe();
  if (FLAGS_use_navigation_mode && !PredictionMap::Ready()) {
    AERROR << "Relative map is empty.";
    return false;
  }

  frame_start_time_ = Clock::NowInSeconds();
  auto end_time1 = std::chrono::system_clock::now();

  // Read localization info. and call OnLocalization to update
  // the PoseContainer.
  localization_reader_->Observe();
  auto ptr_localization_msg = localization_reader_->GetLatestObserved();
  if (ptr_localization_msg == nullptr) {
    AERROR <<"Prediction: cannot receive any localization message.";
    return false;
  }
  auto localization_msg = *ptr_localization_msg;
  OnLocalization(localization_msg);
  auto end_time2 = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_time2 - end_time1;
  ADEBUG << "Time for updating PoseContainer: "
        << diff.count() * 1000 << " msec.";

  // Read planning info. of last frame and call OnPlanning to update
  // the ADCTrajectoryContainer
  planning_reader_->Observe();
  auto ptr_trajectory_msg = planning_reader_->GetLatestObserved();
  if (ptr_trajectory_msg != nullptr) {
    auto trajectory_msg = *ptr_trajectory_msg;
    OnPlanning(trajectory_msg);
  }
  auto end_time3 = std::chrono::system_clock::now();
  diff = end_time3 - end_time2;
  ADEBUG << "Time for updating ADCTrajectoryContainer: "
        << diff.count() * 1000 << " msec.";

  // Get all perception_obstacles of this frame and call OnPerception to
  // process them all.
  auto perception_msg = *perception_obstacles;
  OnPerception(perception_msg);
  auto end_time4 = std::chrono::system_clock::now();
  diff = end_time4 - end_time3;
  ADEBUG << "Time for updating PerceptionContainer: "
        << diff.count() * 1000 << " msec.";

  return true;
}

}  // namespace prediction
}  // namespace apollo
