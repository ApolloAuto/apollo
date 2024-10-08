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
 * @file nano_radar_message_manager.h
 * @brief The class of NanoRadarMessageManager
 */
#pragma once

#include <memory>

#include "modules/common_msgs/sensor_msgs/nano_radar.pb.h"

#include "cyber/cyber.h"
#include "modules/drivers/canbus/can_client/can_client_factory.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"
#include "modules/drivers/radar/nano_radar/protocol/radar_config_200.h"
#include "modules/drivers/radar/nano_radar/protocol/region_config_401.h"

namespace apollo {
namespace drivers {
namespace nano_radar {

class NanoRadarMessageManager
    : public apollo::drivers::canbus::MessageManager<NanoRadar> {
 public:
  explicit NanoRadarMessageManager(
      const std::shared_ptr<apollo::cyber::Writer<NanoRadar>> &writer);
  virtual ~NanoRadarMessageManager() {}
  void set_radar_conf(RadarConf radar_conf);
  apollo::drivers::canbus::ProtocolData<NanoRadar> *GetMutableProtocolDataById(
      const uint32_t message_id);
  void Parse(const uint32_t message_id, const uint8_t *data, int32_t length);
  void set_can_client(
      std::shared_ptr<apollo::drivers::canbus::CanClient> can_client);

 private:
  bool is_configured_ = false;
  RadarConfig200 radar_config_;
  std::shared_ptr<apollo::drivers::canbus::CanClient> can_client_;
  std::shared_ptr<apollo::cyber::Writer<NanoRadar>> nano_radar_writer_;
};

}  // namespace nano_radar
}  // namespace drivers
}  // namespace apollo
