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

/**
 * @file
 */

#include "modules/drivers/radar/racobit_radar/racobit_radar_canbus_component.h"

#include "cyber/common/file.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common_msgs/sensor_msgs/racobit_radar.pb.h"
#include "modules/drivers/radar/racobit_radar/racobit_radar_message_manager.h"

/**
 * @namespace apollo::drivers::racobit_radar
 * @brief apollo::drivers
 */
namespace apollo {
namespace drivers {
namespace racobit_radar {

RacobitRadarCanbusComponent::RacobitRadarCanbusComponent()
    : monitor_logger_buffer_(
          common::monitor::MonitorMessageItem::RACOBIT_RADAR) {}

bool RacobitRadarCanbusComponent::Init() {
  if (!GetProtoConfig(&racobit_radar_conf_)) {
    return OnError("Unable to load canbus conf file: " + ConfigFilePath()).ok();
  }

  AINFO << "The canbus conf file is loaded: " << ConfigFilePath();
  ADEBUG << "Canbus_conf:" << racobit_radar_conf_.ShortDebugString();
  racobit_radar_writer_ =
      node_->CreateWriter<RacobitRadar>(FLAGS_racobit_radar_topic);
  if (!cyber::common::GetProtoFromFile(ConfigFilePath(),
                                       &racobit_radar_conf_)) {
    return OnError("Unable to load canbus conf file: " + ConfigFilePath()).ok();
  }

  AINFO << "The canbus conf file is loaded: " << ConfigFilePath();
  ADEBUG << "Canbus_conf:" << racobit_radar_conf_.ShortDebugString();

  auto can_factory = CanClientFactory::Instance();
  can_factory->RegisterCanClients();
  can_client_ = can_factory->CreateCANClient(
      racobit_radar_conf_.can_conf().can_card_parameter());
  if (!can_client_) {
    return OnError("Failed to create can client.").ok();
  }
  AINFO << "Can client is successfully created.";

  sensor_message_manager_ = std::unique_ptr<RacobitRadarMessageManager>(
      new RacobitRadarMessageManager(racobit_radar_writer_));
  if (sensor_message_manager_ == nullptr) {
    return OnError("Failed to create message manager.").ok();
  }
  sensor_message_manager_->set_radar_conf(racobit_radar_conf_.radar_conf());
  sensor_message_manager_->set_can_client(can_client_);
  AINFO << "Sensor message manager is successfully created.";

  if (can_receiver_.Init(
          can_client_.get(), sensor_message_manager_.get(),
          racobit_radar_conf_.can_conf().enable_receiver_log()) !=
      ErrorCode::OK) {
    return OnError("Failed to init can receiver.").ok();
  }
  AINFO << "The can receiver is successfully initialized.";

  if (can_client_->Start() != ErrorCode::OK) {
    return OnError("Failed to start can client").ok();
  }

  AINFO << "Can client is started.";
  if (ConfigureRadar() != ErrorCode::OK) {
    return OnError("Failed to configure radar.").ok();
  }
  AINFO << "The radar is successfully configured.";

  if (can_receiver_.Start() != ErrorCode::OK) {
    return OnError("Failed to start can receiver.").ok();
  }
  AINFO << "Can receiver is started.";
  monitor_logger_buffer_.INFO("Canbus is started.");

  return true;
}

apollo::common::ErrorCode RacobitRadarCanbusComponent::ConfigureRadar() {
  RadarConfig200 radar_config;
  radar_config.set_radar_conf(racobit_radar_conf_.radar_conf());
  SenderMessage<RacobitRadar> sender_message(RadarConfig200::ID, &radar_config);
  sender_message.Update();
  return can_client_->SendSingleFrame({sender_message.CanFrame()});
}

RacobitRadarCanbusComponent::~RacobitRadarCanbusComponent() {
  if (start_success_) {
    can_receiver_.Stop();
    can_client_->Stop();
  }
}

Status RacobitRadarCanbusComponent::OnError(const std::string &error_msg) {
  monitor_logger_buffer_.ERROR(error_msg);
  return Status(ErrorCode::CANBUS_ERROR, error_msg);
}

}  // namespace racobit_radar
}  // namespace drivers
}  // namespace apollo
