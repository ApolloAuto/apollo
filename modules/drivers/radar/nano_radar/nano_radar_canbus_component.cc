/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/radar/nano_radar/nano_radar_canbus_component.h"

#include "Eigen/Geometry"

#include "modules/common_msgs/sensor_msgs/nano_radar.pb.h"
#include "modules/drivers/canbus/proto/sensor_canbus_conf.pb.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/drivers/canbus/can_client/can_client_factory.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/radar/nano_radar/nano_radar_message_manager.h"

/**
 * @namespace apollo::drivers::nano_radar
 * @brief apollo::drivers
 */
namespace apollo {
namespace drivers {
namespace nano_radar {

using apollo::common::ErrorCode;
using apollo::drivers::canbus::CanClientFactory;
using apollo::drivers::canbus::SenderMessage;
using apollo::drivers::canbus::SensorCanbusConf;

NanoRadarCanbusComponent::NanoRadarCanbusComponent()
    : monitor_logger_buffer_(
          apollo::common::monitor::MonitorMessageItem::NANO_RADAR) {}
NanoRadarCanbusComponent::~NanoRadarCanbusComponent() { Stop(); }

bool NanoRadarCanbusComponent::Init() {
  if (!GetProtoConfig(&nano_radar_conf_)) {
    return OnError("Unable to load canbus conf file: " + ConfigFilePath());
  }

  AINFO << "The canbus conf file is loaded: " << ConfigFilePath();
  ADEBUG << "Canbus_conf:" << nano_radar_conf_.ShortDebugString();

  // Init can client
  auto can_factory = CanClientFactory::Instance();
  can_factory->RegisterCanClients();
  can_client_ = can_factory->CreateCANClient(
      nano_radar_conf_.can_conf().can_card_parameter());
  if (!can_client_) {
    return OnError("Failed to create can client.");
  }
  AINFO << "Can client is successfully created.";
  nano_radar_writer_ =
      node_->CreateWriter<NanoRadar>(nano_radar_conf_.radar_channel());

  sensor_message_manager_ = std::unique_ptr<NanoRadarMessageManager>(
      new NanoRadarMessageManager(nano_radar_writer_));
  if (sensor_message_manager_ == nullptr) {
    return OnError("Failed to create message manager.");
  }
  sensor_message_manager_->set_radar_conf(nano_radar_conf_.radar_conf());
  sensor_message_manager_->set_can_client(can_client_);
  AINFO << "Sensor message manager is successfully created.";

  if (can_receiver_.Init(can_client_.get(), sensor_message_manager_.get(),
                         nano_radar_conf_.can_conf().enable_receiver_log()) !=
      ErrorCode::OK) {
    return OnError("Failed to init can receiver.");
  }
  AINFO << "The can receiver is successfully initialized.";

  start_success_ = Start();
  return start_success_;
}

apollo::common::ErrorCode NanoRadarCanbusComponent::ConfigureRadar() {
  RadarConfig200 radar_config;
  radar_config.set_radar_conf(nano_radar_conf_.radar_conf());
  SenderMessage<NanoRadar> sender_message(RadarConfig200::ID, &radar_config);
  sender_message.Update();
  return can_client_->SendSingleFrame({sender_message.CanFrame()});
}

apollo::common::ErrorCode NanoRadarCanbusComponent::ConfigureRadarRegion() {
  RegionConfig401 radar_config;
  radar_config.set_radar_conf(nano_radar_conf_.radar_conf());
  SenderMessage<NanoRadar> sender_message(RegionConfig401::ID, &radar_config);
  sender_message.Update();
  return can_client_->SendSingleFrame({sender_message.CanFrame()});
}

bool NanoRadarCanbusComponent::Start() {
  // 1. init and start the can card hardware
  if (can_client_->Start() != ErrorCode::OK) {
    return OnError("Failed to start can client");
  }
  AINFO << "Can client is started.";
  if (ConfigureRadar() != ErrorCode::OK) {
    return OnError("Failed to configure radar.");
  }
  AINFO << "The radar is successfully configured.";
  if (ConfigureRadarRegion() != ErrorCode::OK) {
    return OnError("Failed to configure radar region.");
  }
  AINFO << "The radar region is successfully configured.";
  // 2. start receive first then send
  if (can_receiver_.Start() != ErrorCode::OK) {
    return OnError("Failed to start can receiver.");
  }
  AINFO << "Can receiver is started.";

  // last step: publish monitor messages
  monitor_logger_buffer_.INFO("Canbus is started.");

  return true;
}

void NanoRadarCanbusComponent::Stop() {
  if (start_success_) {
    can_receiver_.Stop();
    can_client_->Stop();
  }
}

// Send the error to monitor and return it
bool NanoRadarCanbusComponent::OnError(const std::string& error_msg) {
  monitor_logger_buffer_.ERROR(error_msg);
  AERROR << error_msg;
  return false;
}

}  // namespace nano_radar
}  // namespace drivers
}  // namespace apollo
