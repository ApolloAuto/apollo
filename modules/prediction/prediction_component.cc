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

#include "cyber/common/file.h"
#include "cyber/record/record_reader.h"
#include "cyber/time/clock.h"
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
#include "modules/prediction/proto/prediction_conf.pb.h"
#include "modules/prediction/scenario/scenario_manager.h"
#include "modules/common/util/data_extraction.h"

namespace apollo {
namespace prediction {

using apollo::common::adapter::AdapterConfig;
using apollo::cyber::Clock;
using apollo::perception::PerceptionObstacles;
using apollo::planning::ADCTrajectory;

PredictionComponent::~PredictionComponent() {}

std::string PredictionComponent::Name() const {
  return FLAGS_prediction_module_name;
}

void PredictionComponent::OfflineProcessFeatureProtoFile(
    const std::string& features_proto_file_name) {
  auto obstacles_container_ptr =
      container_manager_->GetContainer<ObstaclesContainer>(
          AdapterConfig::PERCEPTION_OBSTACLES);
  obstacles_container_ptr->Clear();
  Features features;
  apollo::cyber::common::GetProtoFromBinaryFile(features_proto_file_name,
                                                &features);
  for (const Feature& feature : features.feature()) {
    obstacles_container_ptr->InsertFeatureProto(feature);
    Obstacle* obstacle_ptr = obstacles_container_ptr->GetObstacle(feature.id());
    if (!obstacle_ptr) {
      continue;
    }
    evaluator_manager_->EvaluateObstacle(obstacle_ptr, obstacles_container_ptr);
  }
}

bool PredictionComponent::Init() {
  component_start_time_ = Clock::NowInSeconds();

  container_manager_ = std::make_shared<ContainerManager>();
  evaluator_manager_.reset(new EvaluatorManager());
  predictor_manager_.reset(new PredictorManager());
  scenario_manager_.reset(new ScenarioManager());

  PredictionConf prediction_conf;
  if (!ComponentBase::GetProtoConfig(&prediction_conf)) {
    AERROR << "Unable to load prediction conf file: "
           << ComponentBase::ConfigFilePath();
    return false;
  }
  ADEBUG << "Prediction config file is loaded into: "
         << prediction_conf.ShortDebugString();

  if (!MessageProcess::Init(container_manager_.get(), evaluator_manager_.get(),
                            predictor_manager_.get(), prediction_conf)) {
    return false;
  }

  planning_reader_ = node_->CreateReader<ADCTrajectory>(
      prediction_conf.topic_conf().planning_trajectory_topic(), nullptr);

  localization_reader_ =
      node_->CreateReader<localization::LocalizationEstimate>(
          prediction_conf.topic_conf().localization_topic(), nullptr);

  storytelling_reader_ = node_->CreateReader<storytelling::Stories>(
      prediction_conf.topic_conf().storytelling_topic(), nullptr);

  prediction_writer_ = node_->CreateWriter<PredictionObstacles>(
      prediction_conf.topic_conf().prediction_topic());

  container_writer_ = node_->CreateWriter<SubmoduleOutput>(
      prediction_conf.topic_conf().container_topic_name());

  adc_container_writer_ = node_->CreateWriter<ADCTrajectoryContainer>(
      prediction_conf.topic_conf().adccontainer_topic_name());

  perception_obstacles_writer_ = node_->CreateWriter<PerceptionObstacles>(
      prediction_conf.topic_conf().perception_obstacles_topic_name());

  return true;
}

bool PredictionComponent::Proc(
    const std::shared_ptr<PerceptionObstacles>& perception_obstacles) {
  if (FLAGS_use_lego) {
    return ContainerSubmoduleProcess(perception_obstacles);
  }
  return PredictionEndToEndProc(perception_obstacles);
}

bool PredictionComponent::ContainerSubmoduleProcess(
    const std::shared_ptr<PerceptionObstacles>& perception_obstacles) {
  constexpr static size_t kHistorySize = 10;
  const auto frame_start_time = Clock::Now();
  // Read localization info. and call OnLocalization to update
  // the PoseContainer.
  localization_reader_->Observe();
  auto ptr_localization_msg = localization_reader_->GetLatestObserved();
  if (ptr_localization_msg == nullptr) {
    AERROR << "Prediction: cannot receive any localization message.";
    return false;
  }
  MessageProcess::OnLocalization(container_manager_.get(),
                                 *ptr_localization_msg);

  // Read planning info. of last frame and call OnPlanning to update
  // the ADCTrajectoryContainer
  planning_reader_->Observe();
  auto ptr_trajectory_msg = planning_reader_->GetLatestObserved();
  if (ptr_trajectory_msg != nullptr) {
    MessageProcess::OnPlanning(container_manager_.get(), *ptr_trajectory_msg);
  }

  // Read storytelling message and call OnStorytelling to update the
  // StoryTellingContainer
  storytelling_reader_->Observe();
  auto ptr_storytelling_msg = storytelling_reader_->GetLatestObserved();
  if (ptr_storytelling_msg != nullptr) {
    MessageProcess::OnStoryTelling(container_manager_.get(),
                                   *ptr_storytelling_msg);
  }

  MessageProcess::ContainerProcess(container_manager_, *perception_obstacles,
                                   scenario_manager_.get());

  auto obstacles_container_ptr =
      container_manager_->GetContainer<ObstaclesContainer>(
          AdapterConfig::PERCEPTION_OBSTACLES);
  CHECK_NOTNULL(obstacles_container_ptr);

  auto adc_trajectory_container_ptr =
      container_manager_->GetContainer<ADCTrajectoryContainer>(
          AdapterConfig::PLANNING_TRAJECTORY);
  CHECK_NOTNULL(adc_trajectory_container_ptr);

  SubmoduleOutput submodule_output =
      obstacles_container_ptr->GetSubmoduleOutput(kHistorySize,
                                                  frame_start_time);
  submodule_output.set_curr_scenario(scenario_manager_->scenario());
  container_writer_->Write(submodule_output);
  adc_container_writer_->Write(*adc_trajectory_container_ptr);
  perception_obstacles_writer_->Write(*perception_obstacles);
  return true;
}

bool PredictionComponent::PredictionEndToEndProc(
    const std::shared_ptr<PerceptionObstacles>& perception_obstacles) {
  if (FLAGS_prediction_test_mode &&
      (Clock::NowInSeconds() - component_start_time_ >
       FLAGS_prediction_test_duration)) {
    ADEBUG << "Prediction finished running in test mode";
  }

  // Update relative map if needed
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
    AERROR << "Prediction: cannot receive any localization message.";
    return false;
  }
  MessageProcess::OnLocalization(container_manager_.get(),
                                 *ptr_localization_msg);
  auto end_time2 = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_time2 - end_time1;
  ADEBUG << "Time for updating PoseContainer: " << diff.count() * 1000
         << " msec.";

  // Read storytelling message and call OnStorytelling to update the
  // StoryTellingContainer
  storytelling_reader_->Observe();
  auto ptr_storytelling_msg = storytelling_reader_->GetLatestObserved();
  if (ptr_storytelling_msg != nullptr) {
    MessageProcess::OnStoryTelling(container_manager_.get(),
                                   *ptr_storytelling_msg);
  }

  // Read planning info. of last frame and call OnPlanning to update
  // the ADCTrajectoryContainer
  planning_reader_->Observe();
  auto ptr_trajectory_msg = planning_reader_->GetLatestObserved();
  if (ptr_trajectory_msg != nullptr) {
    MessageProcess::OnPlanning(container_manager_.get(), *ptr_trajectory_msg);
  }
  auto end_time3 = std::chrono::system_clock::now();
  diff = end_time3 - end_time2;
  ADEBUG << "Time for updating ADCTrajectoryContainer: " << diff.count() * 1000
         << " msec.";

  // Get all perception_obstacles of this frame and call OnPerception to
  // process them all.
  auto perception_msg = *perception_obstacles;
  PredictionObstacles prediction_obstacles;
  MessageProcess::OnPerception(
      perception_msg, container_manager_, evaluator_manager_.get(),
      predictor_manager_.get(), scenario_manager_.get(), &prediction_obstacles);
  auto end_time4 = std::chrono::system_clock::now();
  diff = end_time4 - end_time3;
  ADEBUG << "Time for updating PerceptionContainer: " << diff.count() * 1000
         << " msec.";

  // Postprocess prediction obstacles message
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
            break;
          }
        }
      }
    }
  }

  auto end_time5 = std::chrono::system_clock::now();
  diff = end_time5 - end_time1;
  ADEBUG << "End to end time elapsed: " << diff.count() * 1000 << " msec.";

  // Publish output
  common::util::FillHeader(node_->Name(), &prediction_obstacles);
  prediction_writer_->Write(prediction_obstacles);

  if (FLAGS_prediction_eval_mode) {
    for (auto const& prediction_obstacle :
          prediction_obstacles.prediction_obstacle()) {
      AINFO << "prediction_eval_log (info): "
            << std::to_string(prediction_obstacle.timestamp()) << ", "
            << prediction_obstacle.perception_obstacle().id() << ", "
            << prediction_obstacle.priority().priority() << ", "
            << prediction_obstacle.perception_obstacle().type() << ", "
            << std::to_string(
              prediction_obstacle.perception_obstacle().position().x())
            << ", " << std::to_string(
              prediction_obstacle.perception_obstacle().position().y());

      if (prediction_obstacle.trajectory_size() != 0) {
        for (auto const& trajectory : prediction_obstacle.trajectory()) {
          for (auto const& trajectory_point : trajectory.trajectory_point()) {
            AINFO << "prediction_eval_log (traj): "
                  << std::to_string(prediction_obstacle.timestamp()) << ", "
                  << prediction_obstacle.perception_obstacle().id() << ", "
                  << std::to_string(trajectory_point.path_point().x()) << ", "
                  << std::to_string(trajectory_point.path_point().y());
          }
          break;
        }
      }
    }
  }

  return true;
}

}  // namespace prediction
}  // namespace apollo
