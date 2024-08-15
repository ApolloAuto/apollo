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

#include "modules/canbus_vehicle/ch/ch_vehicle_factory.h"

#include "cyber/common/log.h"
#include "modules/canbus/common/canbus_gflags.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/util.h"
#include "modules/drivers/canbus/can_client/can_client_factory.h"

using apollo::common::ErrorCode;
using apollo::control::ControlCommand;
using apollo::drivers::canbus::CanClientFactory;

namespace apollo {
namespace canbus {

bool ChVehicleFactory::Init(const CanbusConf *canbus_conf) {
  // Init can client
  auto can_factory = CanClientFactory::Instance();
  can_factory->RegisterCanClients();
  can_client_ = can_factory->CreateCANClient(canbus_conf->can_card_parameter());
  if (!can_client_) {
    AERROR << "Failed to create can client.";
    return false;
  }
  AINFO << "Can client is successfully created.";

  message_manager_ = this->CreateMessageManager();
  if (message_manager_ == nullptr) {
    AERROR << "Failed to create message manager.";
    return false;
  }
  AINFO << "Message manager is successfully created.";

  if (can_receiver_.Init(can_client_.get(), message_manager_.get(),
                         canbus_conf->enable_receiver_log()) != ErrorCode::OK) {
    AERROR << "Failed to init can receiver.";
    return false;
  }
  AINFO << "The can receiver is successfully initialized.";

  if (can_sender_.Init(can_client_.get(), message_manager_.get(),
                       canbus_conf->enable_sender_log()) != ErrorCode::OK) {
    AERROR << "Failed to init can sender.";
    return false;
  }
  AINFO << "The can sender is successfully initialized.";

  vehicle_controller_ = CreateVehicleController();
  if (vehicle_controller_ == nullptr) {
    AERROR << "Failed to create vehicle controller.";
    return false;
  }
  AINFO << "The vehicle controller is successfully created.";

  if (vehicle_controller_->Init(canbus_conf->vehicle_parameter(), &can_sender_,
                                message_manager_.get()) != ErrorCode::OK) {
    AERROR << "Failed to init vehicle controller.";
    return false;
  }

  AINFO << "The vehicle controller is successfully"
        << " initialized with canbus conf as : "
        << canbus_conf->vehicle_parameter().ShortDebugString();

  node_ = ::apollo::cyber::CreateNode("chassis_detail");

  chassis_detail_writer_ =
      node_->CreateWriter<::apollo::canbus::Ch>(FLAGS_chassis_detail_topic);

  chassis_detail_sender_writer_ = node_->CreateWriter<::apollo::canbus::Ch>(
      FLAGS_chassis_detail_sender_topic);

  return true;
}

bool ChVehicleFactory::Start() {
  // 1. init and start the can card hardware
  if (can_client_->Start() != ErrorCode::OK) {
    AERROR << "Failed to start can client";
    return false;
  }
  AINFO << "Can client is started.";

  // 2. start receive first then send
  if (can_receiver_.Start() != ErrorCode::OK) {
    AERROR << "Failed to start can receiver.";
    return false;
  }
  AINFO << "Can receiver is started.";

  // 3. start send
  if (can_sender_.Start() != ErrorCode::OK) {
    AERROR << "Failed to start can sender.";
    return false;
  }

  // 4. start controller
  if (!vehicle_controller_->Start()) {
    AERROR << "Failed to start vehicle controller.";
    return false;
  }

  return true;
}

void ChVehicleFactory::Stop() {
  can_sender_.Stop();
  can_receiver_.Stop();
  can_client_->Stop();
  vehicle_controller_->Stop();
  AINFO << "Cleanup cansender, canreceiver, canclient, vehicle controller.";
}

void ChVehicleFactory::UpdateCommand(
    const apollo::control::ControlCommand *control_command) {
  if (vehicle_controller_->Update(*control_command) != ErrorCode::OK) {
    AERROR << "Failed to process callback function OnControlCommand because "
              "vehicle_controller_->Update error.";
    return;
  }
  can_sender_.Update();
}

void ChVehicleFactory::UpdateCommand(
    const apollo::external_command::ChassisCommand *chassis_command) {
  if (vehicle_controller_->Update(*chassis_command) != ErrorCode::OK) {
    AERROR << "Failed to process callback function OnControlCommand because "
              "vehicle_controller_->Update error.";
    return;
  }
  can_sender_.Update();
}

Chassis ChVehicleFactory::publish_chassis() {
  Chassis chassis = vehicle_controller_->chassis();
  ADEBUG << chassis.ShortDebugString();
  return chassis;
}

void ChVehicleFactory::PublishChassisDetail() {
  Ch chassis_detail = vehicle_controller_->GetNewRecvChassisDetail();
  ADEBUG << "latest chassis_detail is " << chassis_detail.ShortDebugString();
  chassis_detail_writer_->Write(chassis_detail);
}

void ChVehicleFactory::PublishChassisDetailSender() {
  Ch sender_chassis_detail = vehicle_controller_->GetNewSenderChassisDetail();
  ADEBUG << "latest sender_chassis_detail is "
         << sender_chassis_detail.ShortDebugString();
  chassis_detail_sender_writer_->Write(sender_chassis_detail);
}

bool ChVehicleFactory::CheckChassisCommunicationFault() {
  if (vehicle_controller_->CheckChassisCommunicationError()) {
    return true;
  }
  return false;
}

std::unique_ptr<VehicleController<::apollo::canbus::Ch>>
ChVehicleFactory::CreateVehicleController() {
  return std::unique_ptr<VehicleController<::apollo::canbus::Ch>>(
      new ch::ChController());
}

std::unique_ptr<MessageManager<::apollo::canbus::Ch>>
ChVehicleFactory::CreateMessageManager() {
  return std::unique_ptr<MessageManager<::apollo::canbus::Ch>>(
      new ch::ChMessageManager());
}

}  // namespace canbus
}  // namespace apollo
