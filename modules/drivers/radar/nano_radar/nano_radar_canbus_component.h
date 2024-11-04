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

#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/common_msgs/drivers_msgs/can_card_parameter.pb.h"
#include "modules/common_msgs/sensor_msgs/nano_radar.pb.h"

#include "cyber/cyber.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/util/util.h"
#include "modules/drivers/canbus/can_client/can_client.h"
#include "modules/drivers/canbus/can_comm/can_receiver.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"
#include "modules/drivers/radar/nano_radar/nano_radar_message_manager.h"
#include "modules/drivers/radar/nano_radar/protocol/radar_config_200.h"
#include "modules/drivers/radar/nano_radar/protocol/region_config_401.h"

/**
 * @namespace apollo::drivers
 * @brief apollo::drivers
 */
namespace apollo {
namespace drivers {
namespace nano_radar {

/**
 * @class NanoRadarCanbus
 *
 * @brief template of canbus-based sensor module main class (e.g., nano_radar).
 */

class NanoRadarCanbusComponent : public apollo::cyber::Component<> {
 public:
  NanoRadarCanbusComponent();
  ~NanoRadarCanbusComponent();
  bool Init() override;

 private:
  bool OnError(const std::string& error_msg);
  void RegisterCanClients();
  apollo::common::ErrorCode ConfigureRadar();
  apollo::common::ErrorCode ConfigureRadarRegion();
  bool Start();
  void Stop();

  NanoRadarConf nano_radar_conf_;
  std::shared_ptr<apollo::drivers::canbus::CanClient> can_client_;
  apollo::drivers::canbus::CanReceiver<NanoRadar> can_receiver_;
  std::unique_ptr<NanoRadarMessageManager> sensor_message_manager_;
  std::shared_ptr<apollo::cyber::Writer<NanoRadar>> nano_radar_writer_;

  bool start_success_ = false;
  apollo::common::monitor::MonitorLogBuffer monitor_logger_buffer_;
};

CYBER_REGISTER_COMPONENT(NanoRadarCanbusComponent)

}  // namespace nano_radar
}  // namespace drivers
}  // namespace apollo
