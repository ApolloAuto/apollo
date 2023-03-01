/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/canbus_component.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/common/file.h"
#include "cyber/time/time.h"
#include "modules/canbus/common/canbus_gflags.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/util.h"
#include "modules/drivers/canbus/can_client/can_client_factory.h"

using apollo::common::ErrorCode;
using apollo::control::ControlCommand;
using apollo::cyber::Time;
using apollo::cyber::class_loader::ClassLoader;
using apollo::drivers::canbus::CanClientFactory;
using apollo::guardian::GuardianCommand;

namespace apollo {
namespace canbus {

std::string CanbusComponent::Name() const { return FLAGS_canbus_module_name; }

CanbusComponent::CanbusComponent()
    : monitor_logger_buffer_(
          apollo::common::monitor::MonitorMessageItem::CANBUS) {}

bool CanbusComponent::Init() {
  if (!GetProtoConfig(&canbus_conf_)) {
    AERROR << "Unable to load canbus conf file: " << ConfigFilePath();
    return false;
  }
  AINFO << "The canbus conf file is loaded: " << FLAGS_canbus_conf_file;
  ADEBUG << "Canbus_conf:" << canbus_conf_.ShortDebugString();

  if (!apollo::cyber::common::PathExists(FLAGS_load_vehicle_library)) {
    AERROR << FLAGS_load_vehicle_library << " No such vehicle library";
    return false;
  }
  AINFO << "Load the vehicle factory library: " << FLAGS_load_vehicle_library;

  ClassLoader loader(FLAGS_load_vehicle_library);
  auto vehicle_object = loader.CreateClassObj<AbstractVehicleFactory>(
      FLAGS_load_vehicle_class_name);
  if (!vehicle_object) {
    AERROR << "Failed to create the vehicle factory: "
           << FLAGS_load_vehicle_class_name;
    return false;
  }

  vehicle_object_ = vehicle_object;
  if (vehicle_object_ == nullptr) {
    AERROR << "Failed to create vehicle factory pointer.";
  }
  AINFO << "Successfully create vehicle factory: "
        << FLAGS_load_vehicle_class_name;

  if (!vehicle_object_->Init(&canbus_conf_)) {
    AERROR << "Fail to init vehicle factory.";
    return false;
  }
  AINFO << "Vehicle factory is successfully initialized.";

  cyber::ReaderConfig guardian_cmd_reader_config;
  guardian_cmd_reader_config.channel_name = FLAGS_guardian_topic;
  guardian_cmd_reader_config.pending_queue_size =
      FLAGS_guardian_cmd_pending_queue_size;

  cyber::ReaderConfig control_cmd_reader_config;
  control_cmd_reader_config.channel_name = FLAGS_control_command_topic;
  control_cmd_reader_config.pending_queue_size =
      FLAGS_control_cmd_pending_queue_size;

  if (FLAGS_receive_guardian) {
    guardian_cmd_reader_ = node_->CreateReader<GuardianCommand>(
        guardian_cmd_reader_config,
        [this](const std::shared_ptr<GuardianCommand> &cmd) {
          ADEBUG << "Received guardian data: run canbus callback.";
          OnGuardianCommand(*cmd);
        });
  } else {
    control_command_reader_ = node_->CreateReader<ControlCommand>(
        control_cmd_reader_config,
        [this](const std::shared_ptr<ControlCommand> &cmd) {
          ADEBUG << "Received control data: run canbus callback.";
          OnControlCommand(*cmd);
        });
  }

  chassis_writer_ = node_->CreateWriter<Chassis>(FLAGS_chassis_topic);

  if (!vehicle_object_->Start()) {
    AERROR << "Fail to start canclient, cansender, canreceiver, canclient, "
              "vehicle controller.";
    Clear();
    return false;
  }
  AINFO << "Start canclient cansender, canreceiver, canclient, vehicle "
           "controller successfully.";

  monitor_logger_buffer_.INFO("Canbus is started.");

  return true;
}

void CanbusComponent::Clear() {
  vehicle_object_->Stop();
  AINFO << "Cleanup Canbus component";
}

void CanbusComponent::PublishChassis() {
  Chassis chassis = vehicle_object_->publish_chassis();
  common::util::FillHeader(node_->Name(), &chassis);
  chassis_writer_->Write(chassis);
  ADEBUG << chassis.ShortDebugString();
}

bool CanbusComponent::Proc() {
  PublishChassis();
  if (FLAGS_enable_chassis_detail_pub) {
    vehicle_object_->PublishChassisDetail();
  }
  vehicle_object_->UpdateHeartbeat();
  return true;
}

void CanbusComponent::OnControlCommand(const ControlCommand &control_command) {
  int64_t current_timestamp = Time::Now().ToMicrosecond();
  // if command coming too soon, just ignore it.
  if (current_timestamp - last_timestamp_ < FLAGS_min_cmd_interval * 1000) {
    ADEBUG << "Control command comes too soon. Ignore.\n Required "
              "FLAGS_min_cmd_interval["
           << FLAGS_min_cmd_interval << "], actual time interval["
           << current_timestamp - last_timestamp_ << "].";
    return;
  }

  last_timestamp_ = current_timestamp;
  ADEBUG << "Control_sequence_number:"
         << control_command.header().sequence_num() << ", Time_of_delay:"
         << current_timestamp -
                static_cast<int64_t>(control_command.header().timestamp_sec() *
                                     1e6)
         << " micro seconds";

  vehicle_object_->UpdateCommand(&control_command);
}

void CanbusComponent::OnGuardianCommand(
    const GuardianCommand &guardian_command) {
  OnControlCommand(guardian_command.control_command());
}

common::Status CanbusComponent::OnError(const std::string &error_msg) {
  monitor_logger_buffer_.ERROR(error_msg);
  return ::apollo::common::Status(ErrorCode::CANBUS_ERROR, error_msg);
}

}  // namespace canbus
}  // namespace apollo
