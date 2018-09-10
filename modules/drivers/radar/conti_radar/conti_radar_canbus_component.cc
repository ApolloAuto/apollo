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

/**
 * @file
 */

#include "modules/drivers/radar/conti_radar/conti_radar_canbus_component.h"
#include "modules/drivers/proto/conti_radar.pb.h"
#include "modules/drivers/radar/conti_radar/conti_radar_message_manager.h"

/**
 * @namespace apollo::drivers::conti_radar
 * @brief apollo::drivers
 */
namespace apollo {
namespace drivers {
namespace conti_radar {

ContiRadarCanbusComponent::ContiRadarCanbusComponent() {}
ContiRadarCanbusComponent::~ContiRadarCanbusComponent() { Stop(); }

// std::string ContiRadarCanbusComponent::Name() const {
//   return FLAGS_canbus_driver_name;
// }

bool ContiRadarCanbusComponent::Init() {
  if (!GetProtoConfig(&conti_radar_conf_)) {
    return OnError("Unable to load canbus conf file: " + ConfigFilePath());
  }

  AINFO << "The canbus conf file is loaded: " << ConfigFilePath();
  ADEBUG << "Canbus_conf:" << conti_radar_conf_.ShortDebugString();

  // Init can client
  auto can_factory = CanClientFactory::Instance();
  can_factory->RegisterCanClients();
  can_client_ = can_factory->CreateCANClient(
      conti_radar_conf_.can_conf().can_card_parameter());
  if (!can_client_) {
    return OnError("Failed to create can client.");
  }
  AINFO << "Can client is successfully created.";
  conti_radar_writer_ =
      node_->CreateWriter<ContiRadar>("/apollo/sensor/conti_radar");

  sensor_message_manager_ = std::unique_ptr<ContiRadarMessageManager>(
      new ContiRadarMessageManager(conti_radar_writer_));
  if (sensor_message_manager_ == nullptr) {
    return OnError("Failed to create message manager.");
  }
  sensor_message_manager_->set_radar_conf(conti_radar_conf_.radar_conf());
  sensor_message_manager_->set_can_client(can_client_);
  AINFO << "Sensor message manager is successfully created.";

  if (can_receiver_.Init(can_client_.get(), sensor_message_manager_.get(),
                         conti_radar_conf_.can_conf().enable_receiver_log()) !=
      ErrorCode::OK) {
    return OnError("Failed to init can receiver.");
  }
  AINFO << "The can receiver is successfully initialized.";

  return Start();
}

apollo::common::ErrorCode ContiRadarCanbusComponent::ConfigureRadar() {
  RadarConfig200 radar_config;
  radar_config.set_radar_conf(conti_radar_conf_.radar_conf());
  SenderMessage<ContiRadar> sender_message(RadarConfig200::ID, &radar_config);
  sender_message.Update();
  return can_client_->SendSingleFrame({sender_message.CanFrame()});
}

bool ContiRadarCanbusComponent::Start() {
  // 1. init and start the can card hardware
  if (can_client_->Start() != ErrorCode::OK) {
    return OnError("Failed to start can client");
  }
  AINFO << "Can client is started.";
  if (ConfigureRadar() != ErrorCode::OK) {
    return OnError("Failed to configure radar.");
  }
  AINFO << "The radar is successfully configured.";
  // 2. start receive first then send
  if (can_receiver_.Start() != ErrorCode::OK) {
    return OnError("Failed to start can receiver.");
  }
  AINFO << "Can receiver is started.";

  // last step: publish monitor messages
  // apollo::common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
  // buffer.INFO("Canbus is started.");

  return true;
}

void ContiRadarCanbusComponent::Stop() {
  can_receiver_.Stop();
  can_client_->Stop();
}

// Send the error to monitor and return it
bool ContiRadarCanbusComponent::OnError(const std::string &error_msg) {
  // apollo::common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
  // buffer.ERROR(error_msg);
  // return Status(ErrorCode::CANBUS_ERROR, error_msg);
  AERROR << error_msg;
  return false;
}

}  // namespace conti_radar
}  // namespace drivers
}  // namespace apollo
