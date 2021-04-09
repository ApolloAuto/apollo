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
 * @file ultrasonic_radar_message_manager.h
 * @brief The class of UltrasonicRadarMessageManager
 */
#ifndef MODULES_DRIVERS_RADAR_ULTRASONIC_RADAR_MESSAGE_MANAGER_H_
#define MODULES_DRIVERS_RADAR_ULTRASONIC_RADAR_MESSAGE_MANAGER_H_

#include <memory>
#include "modules/drivers/canbus/can_client/can_client_factory.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"
#include "modules/drivers/proto/ultrasonic_radar.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/drivers/canbus/sensor_gflags.h"

namespace apollo {
namespace drivers {
namespace ultrasonic_radar {

using ::apollo::drivers::canbus::ProtocolData;
using ::apollo::common::adapter::AdapterManager;
using ::apollo::drivers::canbus::MessageManager;
using Clock = ::apollo::common::time::Clock;
using micros = std::chrono::microseconds;
using ::apollo::common::ErrorCode;
using apollo::drivers::canbus::CanClient;
using apollo::drivers::canbus::SenderMessage;

class UltrasonicRadarMessageManager : public MessageManager<Ultrasonic> {
 public:
  explicit UltrasonicRadarMessageManager(int entrance_num);
  virtual ~UltrasonicRadarMessageManager() {}
  void Parse(const uint32_t message_id, const uint8_t *data, int32_t length);
  void set_can_client(std::shared_ptr<CanClient> can_client);

 private:
  std::shared_ptr<CanClient> can_client_;
  int entrance_num_;
};

}  // namespace ultrasonic_radar
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_RADAR_ULTRASONIC_RADAR_MESSAGE_MANAGER_H_
