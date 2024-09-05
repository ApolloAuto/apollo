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
#include "modules/planning/planning_component/planning_component.h"

#include "cyber/common/file.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/message_util.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/planning_base/common/history.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/util.h"
#include "modules/planning/planning_component/navi_planning.h"
#include "modules/planning/planning_component/on_lane_planning.h"
namespace apollo {
namespace planning {

using apollo::common::math::Vec2d;
using apollo::cyber::ComponentBase;
using apollo::hdmap::HDMapUtil;

using apollo::control::ControlInteractiveMsg;
using apollo::perception::TrafficLightDetection;
using apollo::relative_map::MapMsg;
using apollo::routing::RoutingRequest;
using apollo::routing::RoutingResponse;
using apollo::storytelling::Stories;

bool PlanningComponent::Init() {
  injector_ = std::make_shared<DependencyInjector>();

  if (FLAGS_use_navigation_mode) {
    planning_base_ = std::make_unique<NaviPlanning>(injector_);
  } else {
    planning_base_ = std::make_unique<OnLanePlanning>(injector_);
  }

  ACHECK(ComponentBase::GetProtoConfig(&config_))
      << "failed to load planning config file "
      << ComponentBase::ConfigFilePath();

  if (FLAGS_planning_offline_learning ||
      config_.learning_mode() != PlanningConfig::NO_LEARNING) {
    if (!message_process_.Init(config_, injector_)) {
      AERROR << "failed to init MessageProcess";
      return false;
    }
  }

  planning_base_->Init(config_);

  planning_command_reader_ = node_->CreateReader<PlanningCommand>(
      config_.topic_config().planning_command_topic(),
      [this](const std::shared_ptr<PlanningCommand>& planning_command) {
        AINFO << "Received planning data: run planning callback."
              << planning_command->header().DebugString();
        std::lock_guard<std::mutex> lock(mutex_);
        planning_command_.CopyFrom(*planning_command);
      });

  traffic_light_reader_ = node_->CreateReader<TrafficLightDetection>(
      config_.topic_config().traffic_light_detection_topic(),
      [this](const std::shared_ptr<TrafficLightDetection>& traffic_light) {
        ADEBUG << "Received traffic light data: run traffic light callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        traffic_light_.CopyFrom(*traffic_light);
      });

  pad_msg_reader_ = node_->CreateReader<PadMessage>(
      config_.topic_config().planning_pad_topic(),
      [this](const std::shared_ptr<PadMessage>& pad_msg) {
        ADEBUG << "Received pad data: run pad callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        pad_msg_.CopyFrom(*pad_msg);
      });

  story_telling_reader_ = node_->CreateReader<Stories>(
      config_.topic_config().story_telling_topic(),
      [this](const std::shared_ptr<Stories>& stories) {
        ADEBUG << "Received story_telling data: run story_telling callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        stories_.CopyFrom(*stories);
      });

  control_interactive_reader_ = node_->CreateReader<ControlInteractiveMsg>(
      config_.topic_config().control_interative_topic(),
      [this](const std::shared_ptr<ControlInteractiveMsg>&
                 control_interactive_msg) {
        ADEBUG << "Received story_telling data: run story_telling callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        control_interactive_msg_.CopyFrom(*control_interactive_msg);
      });

  if (FLAGS_use_navigation_mode) {
    relative_map_reader_ = node_->CreateReader<MapMsg>(
        config_.topic_config().relative_map_topic(),
        [this](const std::shared_ptr<MapMsg>& map_message) {
          ADEBUG << "Received relative map data: run relative map callback.";
          std::lock_guard<std::mutex> lock(mutex_);
          relative_map_.CopyFrom(*map_message);
        });
  }
  planning_writer_ = node_->CreateWriter<ADCTrajectory>(
      config_.topic_config().planning_trajectory_topic());

  rerouting_client_ =
      node_->CreateClient<apollo::external_command::LaneFollowCommand,
                          external_command::CommandStatus>(
          config_.topic_config().routing_request_topic());
  planning_learning_data_writer_ = node_->CreateWriter<PlanningLearningData>(
      config_.topic_config().planning_learning_data_topic());
  command_status_writer_ = node_->CreateWriter<external_command::CommandStatus>(
      FLAGS_planning_command_status);
  return true;
}

bool PlanningComponent::Proc(
    const std::shared_ptr<prediction::PredictionObstacles>&
        prediction_obstacles,
    const std::shared_ptr<canbus::Chassis>& chassis,
    const std::shared_ptr<localization::LocalizationEstimate>&
        localization_estimate) {
  ACHECK(prediction_obstacles != nullptr);

  // check and process possible rerouting request
  CheckRerouting();

  // process fused input data
  local_view_.prediction_obstacles = prediction_obstacles;
  local_view_.chassis = chassis;
  local_view_.localization_estimate = localization_estimate;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!local_view_.planning_command ||
        !common::util::IsProtoEqual(local_view_.planning_command->header(),
                                    planning_command_.header())) {
      local_view_.planning_command =
          std::make_shared<PlanningCommand>(planning_command_);
    }
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.traffic_light =
        std::make_shared<TrafficLightDetection>(traffic_light_);
    local_view_.relative_map = std::make_shared<MapMsg>(relative_map_);
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!local_view_.pad_msg ||
        !common::util::IsProtoEqual(local_view_.pad_msg->header(),
                                    pad_msg_.header())) {
      // Check if "CLEAR_PLANNING" PadMessage is received and process.
      if (pad_msg_.action() == PadMessage::CLEAR_PLANNING) {
        local_view_.planning_command = nullptr;
        planning_command_.Clear();
      }
      local_view_.pad_msg = std::make_shared<PadMessage>(pad_msg_);
    }
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.stories = std::make_shared<Stories>(stories_);
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!local_view_.control_interactive_msg ||
        !common::util::IsProtoEqual(
            local_view_.control_interactive_msg->header(),
            control_interactive_msg_.header())) {
      local_view_.control_interactive_msg =
          std::make_shared<ControlInteractiveMsg>(control_interactive_msg_);
    }
  }

  if (!CheckInput()) {
    AINFO << "Input check failed";
    return false;
  }

  if (config_.learning_mode() != PlanningConfig::NO_LEARNING) {
    // data process for online training
    message_process_.OnChassis(*local_view_.chassis);
    message_process_.OnPrediction(*local_view_.prediction_obstacles);
    if (local_view_.planning_command->has_lane_follow_command()) {
      message_process_.OnRoutingResponse(
          local_view_.planning_command->lane_follow_command());
    }
    message_process_.OnStoryTelling(*local_view_.stories);
    message_process_.OnTrafficLightDetection(*local_view_.traffic_light);
    message_process_.OnLocalization(*local_view_.localization_estimate);
  }

  // publish learning data frame for RL test
  if (config_.learning_mode() == PlanningConfig::RL_TEST) {
    PlanningLearningData planning_learning_data;
    LearningDataFrame* learning_data_frame =
        injector_->learning_based_data()->GetLatestLearningDataFrame();
    if (learning_data_frame) {
      planning_learning_data.mutable_learning_data_frame()->CopyFrom(
          *learning_data_frame);
      common::util::FillHeader(node_->Name(), &planning_learning_data);
      planning_learning_data_writer_->Write(planning_learning_data);
    } else {
      AERROR << "fail to generate learning data frame";
      return false;
    }
    return true;
  }

  ADCTrajectory adc_trajectory_pb;
  planning_base_->RunOnce(local_view_, &adc_trajectory_pb);
  auto start_time = adc_trajectory_pb.header().timestamp_sec();
  common::util::FillHeader(node_->Name(), &adc_trajectory_pb);

  SetLocation(&adc_trajectory_pb);
  // modify trajectory relative time due to the timestamp change in header
  const double dt = start_time - adc_trajectory_pb.header().timestamp_sec();
  for (auto& p : *adc_trajectory_pb.mutable_trajectory_point()) {
    p.set_relative_time(p.relative_time() + dt);
  }
  planning_writer_->Write(adc_trajectory_pb);

  // Send command execution feedback.
  // Error occured while executing the command.
  external_command::CommandStatus command_status;
  common::util::FillHeader(node_->Name(), &command_status);
  if (nullptr != local_view_.planning_command) {
    command_status.set_command_id(local_view_.planning_command->command_id());
  }

  ADCTrajectory::TrajectoryType current_trajectory_type =
      adc_trajectory_pb.trajectory_type();
  if (adc_trajectory_pb.header().status().error_code() !=
      common::ErrorCode::OK) {
    command_status.set_status(external_command::CommandStatusType::ERROR);
    command_status.set_message(adc_trajectory_pb.header().status().msg());
  } else if (planning_base_->IsPlanningFinished(current_trajectory_type)) {
    AINFO << "Set the external_command: FINISHED";
    command_status.set_status(external_command::CommandStatusType::FINISHED);
  } else {
    AINFO << "Set the external_command: RUNNING";
    command_status.set_status(external_command::CommandStatusType::RUNNING);
  }
  command_status_writer_->Write(command_status);

  // record in history
  auto* history = injector_->history();
  history->Add(adc_trajectory_pb);

  return true;
}

void PlanningComponent::CheckRerouting() {
  auto* rerouting = injector_->planning_context()
                        ->mutable_planning_status()
                        ->mutable_rerouting();
  if (!rerouting->need_rerouting()) {
    return;
  }
  common::util::FillHeader(node_->Name(),
                           rerouting->mutable_lane_follow_command());
  auto lane_follow_command_ptr =
      std::make_shared<apollo::external_command::LaneFollowCommand>(
          rerouting->lane_follow_command());
  rerouting_client_->SendRequest(lane_follow_command_ptr);
  rerouting->set_need_rerouting(false);
}

bool PlanningComponent::CheckInput() {
  ADCTrajectory trajectory_pb;

  SetLocation(&trajectory_pb);
  auto* not_ready = trajectory_pb.mutable_decision()
                        ->mutable_main_decision()
                        ->mutable_not_ready();

  if (local_view_.localization_estimate == nullptr) {
    not_ready->set_reason("localization not ready");
  } else if (local_view_.chassis == nullptr) {
    not_ready->set_reason("chassis not ready");
  } else if (HDMapUtil::BaseMapPtr() == nullptr) {
    not_ready->set_reason("map not ready");
  } else {
    // nothing
  }

  if (FLAGS_use_navigation_mode) {
    if (!local_view_.relative_map->has_header()) {
      not_ready->set_reason("relative map not ready");
    }
  } else {
    if (!local_view_.planning_command ||
        !local_view_.planning_command->has_header()) {
      not_ready->set_reason("planning_command not ready");
    }
  }

  if (not_ready->has_reason()) {
    AINFO << not_ready->reason() << "; skip the planning cycle.";
    common::util::FillHeader(node_->Name(), &trajectory_pb);
    planning_writer_->Write(trajectory_pb);
    return false;
  }
  return true;
}

void PlanningComponent::SetLocation(ADCTrajectory* const ptr_trajectory_pb) {
  auto p = ptr_trajectory_pb->mutable_location_pose();
  p->mutable_vehice_location()->set_x(
      local_view_.localization_estimate->pose().position().x());
  p->mutable_vehice_location()->set_y(
      local_view_.localization_estimate->pose().position().y());
  const Vec2d& adc_position = {
      local_view_.localization_estimate->pose().position().x(),
      local_view_.localization_estimate->pose().position().y()};
  Vec2d left_point, right_point;
  if (planning_base_->GenerateWidthOfLane(adc_position, left_point,
                                          right_point)) {
    p->mutable_left_lane_boundary_point()->set_x(left_point.x());
    p->mutable_left_lane_boundary_point()->set_y(left_point.y());
    p->mutable_right_lane_boundary_point()->set_x(right_point.x());
    p->mutable_right_lane_boundary_point()->set_y(right_point.y());
  }
}

}  // namespace planning
}  // namespace apollo
