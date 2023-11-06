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
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/common_msgs/external_command_msgs/action_command.pb.h"
#include "modules/common_msgs/external_command_msgs/chassis_command.pb.h"
#include "modules/common_msgs/external_command_msgs/command_status.pb.h"
#include "modules/common_msgs/external_command_msgs/free_space_command.pb.h"
#include "modules/common_msgs/external_command_msgs/lane_follow_command.pb.h"
#include "modules/common_msgs/external_command_msgs/path_follow_command.pb.h"
#include "modules/common_msgs/external_command_msgs/speed_command.pb.h"
#include "modules/common_msgs/external_command_msgs/valet_parking_command.pb.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/external_command/external_command_demo/proto/demo_config.pb.h"

#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"
#include "modules/common/util/message_util.h"

class ExternalCommandDemo final : public apollo::cyber::TimerComponent {
 public:
  ExternalCommandDemo();

  bool Init() override;

  bool Proc() override;

 private:
  template <typename T>
  void FillCommandHeader(const std::shared_ptr<T>& command);

  void SendActionCommand(
      apollo::external_command::ActionCommandType action_command_type);

  void SendSpeedCommand(double speed);

  void SendSpeedFactorCommand(double speed_factor);

  void RestoreSpeed();

  void SendLaneFollowCommand(
      const std::vector<apollo::external_command::Pose>& way_points,
      const apollo::external_command::Pose& end, double target_speed);

  void SendValetParkingCommand(const std::string& parking_spot_id,
                               double target_speed);

  void SendVehicleSignalCommand();

  void SendCustomChassisCommand();

  void SendPathFollowCommandWithPathRecord(const std::string& record_path);

  void SendPathFollowCommandWithLocationRecord(const std::string& record_dir);

  void CheckCommandStatus(const uint64_t command_id);

  void SendFreespaceCommand(
      const std::vector<apollo::external_command::Point>& way_points,
      const apollo::external_command::Pose& end);

  static void ReadPathFromPathRecord(
      const std::string& record_file,
      google::protobuf::RepeatedPtrField<apollo::external_command::Point>*
          waypoints);

  void ReadPathFromLocationRecord(
      const std::string& record_file,
      google::protobuf::RepeatedPtrField<apollo::external_command::Point>*
          waypoints) const;

  std::shared_ptr<
      apollo::cyber::Client<apollo::external_command::ActionCommand,
                            apollo::external_command::CommandStatus>>
      action_command_client_;
  std::shared_ptr<
      apollo::cyber::Client<apollo::external_command::ChassisCommand,
                            apollo::external_command::CommandStatus>>
      chassis_command_client_;
  std::shared_ptr<
      apollo::cyber::Client<apollo::external_command::FreeSpaceCommand,
                            apollo::external_command::CommandStatus>>
      free_space_command_client_;
  std::shared_ptr<
      apollo::cyber::Client<apollo::external_command::LaneFollowCommand,
                            apollo::external_command::CommandStatus>>
      lane_follow_command_client_;
  std::shared_ptr<
      apollo::cyber::Client<apollo::external_command::PathFollowCommand,
                            apollo::external_command::CommandStatus>>
      path_follow_command_client_;
  std::shared_ptr<
      apollo::cyber::Client<apollo::external_command::SpeedCommand,
                            apollo::external_command::CommandStatus>>
      speed_command_client_;
  std::shared_ptr<
      apollo::cyber::Client<apollo::external_command::ValetParkingCommand,
                            apollo::external_command::CommandStatus>>
      valet_parking_command_client_;
  std::shared_ptr<
      apollo::cyber::Client<apollo::external_command::CommandStatusRequest,
                            apollo::external_command::CommandStatus>>
      status_client_;

  uint64_t command_id_;
  const std::string module_name_;
  apollo::external_command_demo::DemoConfig demo_config_;
};

template <typename T>
void ExternalCommandDemo::FillCommandHeader(const std::shared_ptr<T>& command) {
  apollo::common::util::FillHeader(module_name_, command.get());
  command->set_command_id(++command_id_);
}

CYBER_REGISTER_COMPONENT(ExternalCommandDemo);
