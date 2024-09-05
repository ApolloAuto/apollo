/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
 * @file routing_command_processor_base.h
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/common_msgs/external_command_msgs/command_status.pb.h"
#include "modules/common_msgs/planning_msgs/planning_command.pb.h"
#include "modules/common_msgs/routing_msgs/routing.pb.h"
#include "modules/external_command/command_processor/command_processor_base/proto/command_processor_config.pb.h"

#include "cyber/cyber.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/external_command/command_processor/command_processor_base/command_processor_base.h"
#include "modules/external_command/command_processor/command_processor_base/util/lane_way_tool.h"
#include "modules/external_command/command_processor/command_processor_base/util/message_reader.h"
#include "modules/external_command/command_processor/command_processor_base/util/message_writer.h"
#include "modules/routing/routing.h"

/**
 * @namespace apollo::external_command
 * @brief apollo::external_command
 */
namespace apollo {
namespace external_command {

/**
 * @class ExternalCommandProcessComponent
 *
 * @brief The external interface for processing external commands.
 */
template <typename T>
class MotionCommandProcessorBase : public CommandProcessorBase {
 public:
  MotionCommandProcessorBase();

  ~MotionCommandProcessorBase() override = default;

  bool Init(const std::shared_ptr<cyber::Node>& node) override;
  /**
   * @brief Get the command status.
   * @param command_id Id of the command.
   * @param status Output command status.
   */
  bool GetCommandStatus(int64_t command_id,
                        CommandStatus* status) const override;

 protected:
  const std::shared_ptr<apollo::external_command::LaneWayTool>& GetLaneWayTool()
      const {
    return lane_way_tool_;
  }
  /**
   * @brief Convert moving command to RoutingRequest.
   * @param command moving command to be converted.
   * @param routing_request Convert result of RoutingRequest. If there is no
   * LaneFollow action in moving command, the convert result can be nullptr.
   * @return Return true if there is error occurs for converting.
   */
  virtual bool Convert(const std::shared_ptr<T>& command,
                       std::shared_ptr<apollo::routing::RoutingRequest>&
                           routing_request) const = 0;
  /**
   * @brief Set the start pose of lane_way_points with current vehicle pose.
   * @param lane_way_points start pose to be updated.
   * @return Return true if there is error occurs.
   */
  bool SetStartPose(
      std::vector<apollo::routing::LaneWaypoint>* lane_way_points) const;
  /**
   * @brief Set the start pose of RoutingRequest with current vehicle pose.
   * @param routing_request RoutingRequest to be updated.
   * @return Return true if there is error occurs.
   */
  bool SetStartPose(
      std::shared_ptr<apollo::routing::RoutingRequest>& routing_request) const;
  /**
   * @brief Process the incoming lane follow command. Search the routing to end
   * point and send to planning module.
   * @param command Incoming lane follow command.
   * @param status The command process result before sending to planning module.
   */
  void OnCommand(const std::shared_ptr<T>& command,
                 std::shared_ptr<CommandStatus>& status);
  /**
   * @brief Process special command except RoutingRequest.
   * @param command RoutingCommand to be converted.
   * @param planning_command Output process result.
   * @return True if no error occurs.
   */
  virtual bool ProcessSpecialCommand(
      const std::shared_ptr<T>& command,
      const std::shared_ptr<apollo::planning::PlanningCommand>&
          planning_command) const = 0;

  std::shared_ptr<cyber::Service<T, CommandStatus>> command_service_;
  std::shared_ptr<WriterHandle> planning_command_writer_;
  std::shared_ptr<WriterHandle> routing_response_writer_;

  std::shared_ptr<apollo::routing::Routing> routing_;
  std::shared_ptr<apollo::external_command::LaneWayTool> lane_way_tool_;
  T last_received_command_;
  MessageReader* message_reader_;
  std::string planning_command_status_name_;

 private:
  std::mutex mutex_;
};

template <typename T>
MotionCommandProcessorBase<T>::MotionCommandProcessorBase()
    : command_service_(nullptr),
      planning_command_writer_(nullptr),
      routing_(std::make_shared<apollo::routing::Routing>()),
      lane_way_tool_(nullptr),
      message_reader_(MessageReader::Instance()) {
  last_received_command_.set_command_id(-1);
}

template <typename T>
bool MotionCommandProcessorBase<T>::Init(
    const std::shared_ptr<cyber::Node>& node) {
  // Init function of base class should be invoked first.
  if (!CommandProcessorBase::Init(node)) {
    AERROR << "MotionCommandProcessorBase init failed!";
    return false;
  }
  lane_way_tool_ =
      std::make_shared<apollo::external_command::LaneWayTool>(node);
  routing_->Init();
  // Create service for input command.
  const auto& config = GetProcessorConfig();
  command_service_ = node->CreateService<T, CommandStatus>(
      config.input_command_name(),
      [this](const std::shared_ptr<T>& command,
             std::shared_ptr<CommandStatus>& status) {
        this->OnCommand(command, status);
      });

  CHECK_GT(config.output_command_name().size(), 0);
  // Create writer for output command.
  apollo::cyber::proto::RoleAttributes attr;
  attr.set_channel_name(config.output_command_name().Get(0));
  auto qos = attr.mutable_qos_profile();
  qos->set_history(apollo::cyber::proto::QosHistoryPolicy::HISTORY_KEEP_LAST);
  qos->set_reliability(
      apollo::cyber::proto::QosReliabilityPolicy::RELIABILITY_RELIABLE);
  qos->set_durability(
      apollo::cyber::proto::QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL);
  auto message_writer = MessageWriter::Instance();
  planning_command_writer_ =
      message_writer->RegisterMessage<apollo::planning::PlanningCommand>(
          attr, config.planning_command_history_name());
  routing_response_writer_ =
      message_writer->RegisterMessage<apollo::routing::RoutingResponse>(
          FLAGS_routing_response_topic);
  // Create reader for input command status.
  CHECK_GT(config.input_command_status_name().size(), 0);
  planning_command_status_name_ = config.input_command_status_name().Get(0);
  message_reader_->RegisterMessage<CommandStatus>(
      planning_command_status_name_);
  return true;
}

template <typename T>
bool MotionCommandProcessorBase<T>::GetCommandStatus(
    int64_t command_id, CommandStatus* status) const {
  CHECK_NOTNULL(status);
  if (last_received_command_.command_id() == command_id) {
    status->set_command_id(command_id);
    auto* latest_planning_command_status =
        message_reader_->GetMessage<CommandStatus>(
            planning_command_status_name_);
    if (nullptr == latest_planning_command_status) {
      status->set_status(apollo::external_command::CommandStatusType::ERROR);
      status->set_message("Cannot get planning command status!");
      return true;
    }
    status->CopyFrom(*latest_planning_command_status);
    return true;
  }
  return false;
}

template <typename T>
void MotionCommandProcessorBase<T>::OnCommand(
    const std::shared_ptr<T>& command, std::shared_ptr<CommandStatus>& status) {
  CHECK_NOTNULL(command);
  CHECK_NOTNULL(status);
  if (command->has_header()) {
    double timestamp = apollo::cyber::Clock::NowInSeconds();
    AINFO << std::setprecision(12) << "timestamp: " << timestamp << " "
          << "request for " << command->header().timestamp_sec();
    if (timestamp - command->header().timestamp_sec() > 2.0) {
      AINFO << "request for " << command->header().module_name()
            << " has been timeouted";
      return;
    }
  }
  last_received_command_.CopyFrom(*command);
  status->set_command_id(command->command_id());
  // Convert command to RoutingRequest.
  std::shared_ptr<apollo::routing::RoutingRequest> routing_request = nullptr;
  bool convert_result = Convert(command, routing_request);
  if (!convert_result) {
    status->set_status(CommandStatusType::ERROR);
    status->set_message("Cannot convert command to RoutingRequest: " +
                        command->DebugString());
    return;
  }
  // Search routing.
  std::string module_name = "UNKNOWN";
  if (command->has_header()) {
    module_name = command->header().module_name();
  }
  std::lock_guard<std::mutex> guard(this->mutex_);
  auto planning_command = std::make_shared<apollo::planning::PlanningCommand>();
  common::util::FillHeader(module_name, planning_command.get());
  if (nullptr != routing_request) {
    AINFO << "Process routing: " << routing_request->DebugString();
    auto routing_response =
        std::make_shared<apollo::routing::RoutingResponse>();
    if (!routing_->Process(routing_request, routing_response.get())) {
      status->set_status(CommandStatusType::ERROR);
      status->set_message("Cannot get routing of command: " +
                          command->DebugString());
      AERROR << "Routing process failed! ";
      return;
    }
    planning_command->mutable_lane_follow_command()->CopyFrom(
        *routing_response);
    // Publish routing response to display on HMI.
    routing_response_writer_->Write(routing_response);
  }
  // Process command except RoutingRequest.
  if (!ProcessSpecialCommand(command, planning_command)) {
    AERROR << "Process special command failed!";
    return;
  }
  planning_command->set_command_id(command->command_id());
  // Set target_speed.
  if (command->has_target_speed()) {
    planning_command->set_target_speed(command->target_speed());
  }
  planning_command->set_is_motion_command(true);
  // Send routing response to planning module.
  planning_command_writer_->Write(planning_command);
  AINFO << "publish: " << planning_command->DebugString();
  status->set_status(CommandStatusType::RUNNING);
}

template <typename T>
bool MotionCommandProcessorBase<T>::SetStartPose(
    std::vector<apollo::routing::LaneWaypoint>* lane_way_points) const {
  CHECK_NOTNULL(lane_way_points);
  // Get the current vehicle pose as start pose.
  if (!lane_way_tool_->GetVehicleLaneWayPoints(lane_way_points)) {
    AERROR << "Get lane near start pose failed!";
    return false;
  }
  return true;
}

template <typename T>
bool MotionCommandProcessorBase<T>::SetStartPose(
    std::shared_ptr<apollo::routing::RoutingRequest>& routing_request) const {
  CHECK_NOTNULL(routing_request);
  // Get the current vehicle pose as start pose.
  auto start_pose = routing_request->add_waypoint();
  if (!lane_way_tool_->GetVehicleLaneWayPoint(start_pose)) {
    AERROR << "Get lane near start pose failed!";
    return false;
  }
  return true;
}

}  // namespace external_command
}  // namespace apollo
