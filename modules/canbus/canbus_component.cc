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
using apollo::external_command::ChassisCommand;
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
  AINFO << "Successfully create vehicle factory: "
        << FLAGS_load_vehicle_class_name;

  vehicle_object_ = vehicle_object;
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

  cyber::ReaderConfig chassis_cmd_reader_config;
  chassis_cmd_reader_config.channel_name = FLAGS_chassis_command_topic;
  chassis_cmd_reader_config.pending_queue_size =
      FLAGS_control_cmd_pending_queue_size;

  // init cmd reader
  if (FLAGS_receive_guardian) {
    guardian_cmd_reader_ = node_->CreateReader<GuardianCommand>(
        guardian_cmd_reader_config,
        [this](const std::shared_ptr<GuardianCommand> &cmd) {
          ADEBUG << "Received guardian data: run canbus callback.";
          const auto start_time = Time::Now().ToMicrosecond();
          OnGuardianCommand(*cmd);
          const auto end_time = Time::Now().ToMicrosecond();
          if ((end_time - start_time) * 1e-6 > FLAGS_guardian_period) {
            AWARN << "Guardian callback time: "
                  << (end_time - start_time) * 1e-3 << " ms.";
          }
        });
  } else {
    control_command_reader_ = node_->CreateReader<ControlCommand>(
        control_cmd_reader_config,
        [this](const std::shared_ptr<ControlCommand> &cmd) {
          ADEBUG << "Received control data: run canbus callback.";
          const auto start_time = Time::Now().ToMicrosecond();
          OnControlCommand(*cmd);
          const auto end_time = Time::Now().ToMicrosecond();
          if ((end_time - start_time) * 1e-6 > FLAGS_control_period) {
            AWARN << "Control callback time: " << (end_time - start_time) * 1e-3
                  << " ms.";
          }
        });
  }

  // init chassis cmd reader
  chassis_command_reader_ = node_->CreateReader<ChassisCommand>(
      chassis_cmd_reader_config,
      [this](const std::shared_ptr<ChassisCommand> &cmd) {
        ADEBUG << "Received control data: run canbus callback.";
        OnChassisCommand(*cmd);
      });

  // init chassis writer
  chassis_writer_ = node_->CreateWriter<Chassis>(FLAGS_chassis_topic);

  // start canbus vehicle
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
  if (is_control_cmd_time_delay_) {
    AERROR << "cmd time delay";
    chassis.set_error_code(Chassis::CMD_NOT_IN_PERIOD);
  }
  common::util::FillHeader(node_->Name(), &chassis);
  chassis_writer_->Write(chassis);
  ADEBUG << chassis.ShortDebugString();
}

bool CanbusComponent::Proc() {
  const auto start_time = Time::Now().ToMicrosecond();

  if (FLAGS_receive_guardian) {
    guardian_cmd_reader_->Observe();
    const auto &guardian_cmd_msg = guardian_cmd_reader_->GetLatestObserved();
    if (guardian_cmd_msg == nullptr) {
      AERROR << "guardian cmd msg is not ready!";
    } else {
      OnGuardianCommandCheck(*guardian_cmd_msg);
    }
  } else {
    control_command_reader_->Observe();
    const auto &control_cmd_msg = control_command_reader_->GetLatestObserved();
    if (control_cmd_msg == nullptr) {
      AERROR << "control cmd msg is not ready!";
    } else {
      OnControlCommandCheck(*control_cmd_msg);
    }
  }

  // check can receiver msg lost
  if (vehicle_object_->CheckChassisCommunicationFault()) {
    AERROR << "Can not get the chassis info, please check the chassis "
              "communication!";
    is_chassis_communication_fault_ = true;
  } else {
    is_chassis_communication_fault_ = false;
  }

  // publish "/apollo/canbus/chassis"
  PublishChassis();

  // publish "/apollo/canbus/chassis_detail"
  if (FLAGS_enable_chassis_detail_pub) {
    vehicle_object_->PublishChassisDetail();
  }

  // publish "/apollo/canbus/chassis_detail_sender"
  if (FLAGS_enable_chassis_detail_sender_pub) {
    vehicle_object_->PublishChassisDetailSender();
  }

  // update heartbeat in can sender
  vehicle_object_->UpdateHeartbeat();

  const auto end_time = Time::Now().ToMicrosecond();
  const double time_diff_ms = (end_time - start_time) * 1e-3;
  if (time_diff_ms > (1 / FLAGS_chassis_freq * 1e3)) {
    AWARN << "CanbusComponent::Proc() takes too much time: " << time_diff_ms
          << " ms";
  }

  return true;
}

void CanbusComponent::OnControlCommand(const ControlCommand &control_command) {
  // us : microsecord = 1e-3 millisecond = 1e-6 second
  double current_timestamp = Time::Now().ToMicrosecond();
  // if command coming too soon, just ignore it.
  // us < 5 ms(millisecond) *1000 (=5000us microsecord)
  if (current_timestamp - last_timestamp_controlcmd_ <
      FLAGS_min_cmd_interval * 1000) {
    ADEBUG << "Control command comes too soon. Ignore. Required "
              "FLAGS_min_cmd_interval["
           << FLAGS_min_cmd_interval << "] ms, actual time interval["
           << (current_timestamp - last_timestamp_controlcmd_) * 1e-3
           << "] ms.";
    return;
  }
  last_timestamp_controlcmd_ = current_timestamp;

  if (!is_control_cmd_time_delay_) {
    vehicle_object_->UpdateCommand(&control_command);
  }
}

void CanbusComponent::OnControlCommandCheck(
    const ControlCommand &control_command) {
  // us : microsecord = 1e-3 millisecond = 1e-6 second
  double current_timestamp = Time::Now().ToMicrosecond();
  // cmd_time_diff: s
  double cmd_time_diff =
      current_timestamp * 1e-6 - control_command.header().timestamp_sec();
  if ((FLAGS_use_control_cmd_check &&
       (cmd_time_diff > (FLAGS_max_control_miss_num * FLAGS_control_period))) ||
      is_chassis_communication_fault_) {
    AERROR_IF(cmd_time_diff >
              (FLAGS_max_control_miss_num * FLAGS_control_period))
        << "Control cmd timeout, sequence_number:"
        << control_command.header().sequence_num()
        << ", Time_of_delay:" << cmd_time_diff << " s"
        << ", time delay threshold: "
        << (FLAGS_max_control_miss_num * FLAGS_control_period) << " s"
        << ", need to process cmd timeout.";
    AERROR_IF(is_chassis_communication_fault_)
        << "Chassis communication fault, need to process cmd timeout.";

    if (vehicle_object_->Driving_Mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        vehicle_object_->Driving_Mode() == Chassis::AUTO_STEER_ONLY ||
        vehicle_object_->Driving_Mode() == Chassis::AUTO_SPEED_ONLY ||
        FLAGS_chassis_debug_mode || is_chassis_communication_fault_) {
      is_control_cmd_time_delay_ = true;
      GuardianCommand new_guardian_command;
      new_guardian_command.mutable_control_command()->CopyFrom(control_command);
      ProcessGuardianCmdTimeout(&new_guardian_command);
      ADEBUG << "new_guardian_command is "
             << new_guardian_command.ShortDebugString();
      vehicle_object_->UpdateCommand(&new_guardian_command.control_command());
    }
  } else {
    is_control_cmd_time_delay_ = false;
  }
}

void CanbusComponent::OnGuardianCommand(
    const GuardianCommand &guardian_command) {
  if (!is_control_cmd_time_delay_) {
    OnControlCommand(guardian_command.control_command());
  }
}

void CanbusComponent::OnGuardianCommandCheck(
    const GuardianCommand &guardian_command) {
  // us : microsecord = 1e-3 millisecond = 1e-6 second
  double current_timestamp = Time::Now().ToMicrosecond();
  // cmd_time_diff: s
  double guardian_cmd_time_diff =
      current_timestamp * 1e-6 - guardian_command.header().timestamp_sec();
  if ((FLAGS_use_guardian_cmd_check &&
       (guardian_cmd_time_diff >
        (FLAGS_max_guardian_miss_num * FLAGS_guardian_period))) ||
      is_chassis_communication_fault_) {
    AERROR << "Guardain cmd timeout, sequence_number:"
           << guardian_command.header().sequence_num()
           << ", Time_of_delay:" << guardian_cmd_time_diff << " s"
           << ", time delay threshold: "
           << (FLAGS_max_guardian_miss_num * FLAGS_guardian_period) << " s"
           << ", need to process cmd timeout.";

    if (vehicle_object_->Driving_Mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        vehicle_object_->Driving_Mode() == Chassis::AUTO_STEER_ONLY ||
        vehicle_object_->Driving_Mode() == Chassis::AUTO_SPEED_ONLY ||
        is_chassis_communication_fault_) {
      is_control_cmd_time_delay_ = true;
      GuardianCommand new_guardian_command;
      new_guardian_command.CopyFrom(guardian_command);
      ProcessGuardianCmdTimeout(&new_guardian_command);
      ADEBUG << "new_guardian_command is "
             << new_guardian_command.ShortDebugString();
      vehicle_object_->UpdateCommand(&new_guardian_command.control_command());
    }
  } else {
    is_control_cmd_time_delay_ = false;
  }
}

void CanbusComponent::OnChassisCommand(const ChassisCommand &chassis_command) {
  // us : microsecord = 1e-3 millisecond = 1e-6 second
  int64_t current_timestamp = Time::Now().ToMicrosecond();
  // if command coming too soon, just ignore it.
  // us < 5 ms(millisecond) *1000 (=5000us microsecord)
  if (current_timestamp - last_timestamp_chassiscmd_ <
      FLAGS_min_cmd_interval * 1000) {
    ADEBUG << "Control command comes too soon. Ignore.\n Required "
              "FLAGS_min_cmd_interval["
           << FLAGS_min_cmd_interval << "], actual time interval["
           << current_timestamp - last_timestamp_chassiscmd_ << "].";
    return;
  }
  last_timestamp_chassiscmd_ = current_timestamp;

  ADEBUG << "Control_sequence_number:"
         << chassis_command.header().sequence_num() << ", Time_of_delay:"
         << current_timestamp -
                static_cast<int64_t>(chassis_command.header().timestamp_sec() *
                                     1e6)
         << " micro seconds";

  vehicle_object_->UpdateCommand(&chassis_command);
}

common::Status CanbusComponent::OnError(const std::string &error_msg) {
  monitor_logger_buffer_.ERROR(error_msg);
  return ::apollo::common::Status(ErrorCode::CANBUS_ERROR, error_msg);
}

void CanbusComponent::ProcessTimeoutByClearCanSender() {
  if (vehicle_object_->Driving_Mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      vehicle_object_->Driving_Mode() != Chassis::AUTO_STEER_ONLY &&
      vehicle_object_->Driving_Mode() != Chassis::AUTO_SPEED_ONLY &&
      !FLAGS_chassis_debug_mode) {
    ADEBUG << "The current driving mode does not need to check cmd timeout.";
    if (vehicle_object_->IsSendProtocolClear()) {
      AINFO << "send protocol is clear, ignore driving mode, need to recover "
               "send protol.";
      vehicle_object_->AddSendProtocol();
    }
    return;
  }

  if (!is_control_cmd_time_delay_previous_ && is_control_cmd_time_delay_) {
    AINFO << "control cmd time latency delay, clear send protocol.";
    vehicle_object_->ClearSendProtocol();
  } else if (is_control_cmd_time_delay_previous_ &&
             !is_control_cmd_time_delay_) {
    AINFO << "control cmd time latency reover, add send protocol.";
    if (vehicle_object_->IsSendProtocolClear()) {
      vehicle_object_->AddSendProtocol();
    }
  }
  is_control_cmd_time_delay_previous_ = is_control_cmd_time_delay_;
}

void CanbusComponent::ProcessGuardianCmdTimeout(
    GuardianCommand *guardian_command) {
  AINFO << "Into cmd timeout process, set estop.";
  guardian_command->mutable_control_command()->set_throttle(0.0);
  guardian_command->mutable_control_command()->set_steering_target(0.0);
  guardian_command->mutable_control_command()->set_steering_rate(25.0);
  guardian_command->mutable_control_command()->set_brake(FLAGS_estop_brake);
}

}  // namespace canbus
}  // namespace apollo
