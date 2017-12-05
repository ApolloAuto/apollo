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

#include "modules/canbus/canbus.h"

#include "modules/canbus/common/canbus_gflags.h"
#include "modules/canbus/vehicle/vehicle_factory.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/adapters/proto/adapter_config.pb.h"
#include "modules/common/time/time.h"
#include "modules/common/util/util.h"
#include "modules/drivers/canbus/can_client/can_client_factory.h"

namespace apollo {
namespace canbus {

using apollo::common::adapter::AdapterConfig;
using apollo::common::adapter::AdapterManager;
using apollo::common::monitor::MonitorMessageItem;
using apollo::common::Status;
using apollo::common::ErrorCode;
using apollo::drivers::canbus::CanClientFactory;

using apollo::control::ControlCommand;
using apollo::common::time::Clock;

std::string Canbus::Name() const { return FLAGS_canbus_module_name; }

Status Canbus::Init() {
  // load conf
  if (!common::util::GetProtoFromFile(FLAGS_canbus_conf_file, &canbus_conf_)) {
    return OnError("Unable to load canbus conf file: " +
                   FLAGS_canbus_conf_file);
  }

  AdapterManager::Init(FLAGS_adapter_config_filename);
  AINFO << "The adapter manager is successfully initialized.";

  AINFO << "The canbus conf file is loaded: " << FLAGS_canbus_conf_file;
  ADEBUG << "Canbus_conf:" << canbus_conf_.ShortDebugString();

  // Init can client
  auto *can_factory = CanClientFactory::instance();
  can_factory->RegisterCanClients();
  can_client_ = can_factory->CreateCANClient(canbus_conf_.can_card_parameter());
  if (!can_client_) {
    return OnError("Failed to create can client.");
  }
  AINFO << "Can client is successfully created.";

  VehicleFactory vehicle_factory;
  vehicle_factory.RegisterVehicleFactory();
  auto vehicle_object =
      vehicle_factory.CreateVehicle(canbus_conf_.vehicle_parameter());
  if (!vehicle_object) {
    return OnError("Failed to create vehicle:");
  }

  message_manager_ = vehicle_object->CreateMessageManager();
  if (message_manager_ == nullptr) {
    return OnError("Failed to create message manager.");
  }
  AINFO << "Message manager is successfully created.";

  if (can_receiver_.Init(can_client_.get(), message_manager_.get(),
                         canbus_conf_.enable_receiver_log()) != ErrorCode::OK) {
    return OnError("Failed to init can receiver.");
  }
  AINFO << "The can receiver is successfully initialized.";

  if (can_sender_.Init(can_client_.get(), canbus_conf_.enable_sender_log()) !=
      ErrorCode::OK) {
    return OnError("Failed to init can sender.");
  }
  AINFO << "The can sender is successfully initialized.";

  vehicle_controller_ = vehicle_object->CreateVehicleController();
  if (vehicle_controller_ == nullptr) {
    return OnError("Failed to create vehicle controller.");
  }
  AINFO << "The vehicle controller is successfully created.";

  if (vehicle_controller_->Init(canbus_conf_.vehicle_parameter(), &can_sender_,
                                message_manager_.get()) != ErrorCode::OK) {
    return OnError("Failed to init vehicle controller.");
  }
  AINFO << "The vehicle controller is successfully initialized.";

  return Status::OK();
}

Status Canbus::Start() {
  // 1. init and start the can card hardware
  if (can_client_->Start() != ErrorCode::OK) {
    return OnError("Failed to start can client");
  }
  AINFO << "Can client is started.";

  // 2. start receive first then send
  if (can_receiver_.Start() != ErrorCode::OK) {
    return OnError("Failed to start can receiver.");
  }
  AINFO << "Can receiver is started.";

  // 3. start send
  if (can_sender_.Start() != ErrorCode::OK) {
    return OnError("Failed to start can sender.");
  }

  // 4. start controller
  if (vehicle_controller_->Start() == false) {
    return OnError("Failed to start vehicle controller.");
  }

  // 5. set timer to triger publish info periodly
  const double duration = 1.0 / FLAGS_chassis_freq;
  timer_ = AdapterManager::CreateTimer(ros::Duration(duration),
                                       &Canbus::OnTimer, this);
  AdapterManager::AddControlCommandCallback(&Canbus::OnControlCommand, this);

  // last step: publish monitor messages
  apollo::common::monitor::MonitorBuffer buffer(&monitor_);
  buffer.INFO("Canbus is started.");

  return Status::OK();
}

void Canbus::PublishChassis() {
  Chassis chassis = vehicle_controller_->chassis();
  AdapterManager::FillChassisHeader(FLAGS_canbus_node_name, &chassis);

  AdapterManager::PublishChassis(chassis);
  ADEBUG << chassis.ShortDebugString();
}

void Canbus::PublishChassisDetail() {
  ChassisDetail chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);
  ADEBUG << chassis_detail.ShortDebugString();

  AdapterManager::PublishChassisDetail(chassis_detail);
}

void Canbus::OnTimer(const ros::TimerEvent &) {
  PublishChassis();
  if (FLAGS_enable_chassis_detail_pub) {
    PublishChassisDetail();
  }
}

void Canbus::Stop() {
  timer_.stop();

  can_sender_.Stop();
  can_receiver_.Stop();
  can_client_->Stop();
  vehicle_controller_->Stop();
}

void Canbus::OnControlCommand(const ControlCommand &control_command) {
  int64_t current_timestamp =
      apollo::common::time::AsInt64<common::time::micros>(Clock::Now());
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
         << current_timestamp - control_command.header().timestamp_sec();

  if (vehicle_controller_->Update(control_command) != ErrorCode::OK) {
    AERROR << "Failed to process callback function OnControlCommand because "
              "vehicle_controller_->Update error.";
    return;
  }
  can_sender_.Update();
}

// Send the error to monitor and return it
Status Canbus::OnError(const std::string &error_msg) {
  apollo::common::monitor::MonitorBuffer buffer(&monitor_);
  buffer.ERROR(error_msg);
  return Status(ErrorCode::CANBUS_ERROR, error_msg);
}

}  // namespace canbus
}  // namespace apollo
