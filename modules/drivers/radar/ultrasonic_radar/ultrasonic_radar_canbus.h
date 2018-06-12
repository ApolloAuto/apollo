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

#ifndef MODULES_DRIVERS_RADAR_ULTRASONIC_RADAR_ULTRASONIC_RADAR_CANBUS_H_
#define MODULES_DRIVERS_RADAR_ULTRASONIC_RADAR_ULTRASONIC_RADAR_CANBUS_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ros/include/ros/ros.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/adapters/proto/adapter_config.pb.h"
#include "modules/common/apollo_app.h"
#include "modules/common/macro.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/time/time.h"
#include "modules/common/util/util.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/drivers/canbus/can_client/can_client.h"
#include "modules/drivers/canbus/can_client/can_client_factory.h"
#include "modules/drivers/canbus/can_comm/can_receiver.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"
#include "modules/drivers/canbus/proto/can_card_parameter.pb.h"
#include "modules/drivers/canbus/proto/sensor_canbus_conf.pb.h"
#include "modules/drivers/canbus/sensor_gflags.h"
#include "modules/drivers/radar/ultrasonic_radar/ultrasonic_radar_message_manager.h"
#include "modules/drivers/radar/ultrasonic_radar/proto/ultrasonic_radar_conf.pb.h"
#include "modules/drivers/proto/ultrasonic_radar.pb.h"

/**
 * @namespace apollo::drivers
 * @brief apollo::drivers
 */
namespace apollo {
namespace drivers {
namespace ultrasonic_radar {

/**
* @class UltrasonicRadarCanbus
*
* @brief template of canbus-based sensor module main class (e.g., ultrasonic_radar).
*/

using apollo::common::adapter::AdapterConfig;
using apollo::common::adapter::AdapterManager;
using apollo::common::monitor::MonitorMessageItem;
using apollo::common::Status;
using apollo::common::ErrorCode;
using apollo::common::time::Clock;
using apollo::drivers::canbus::CanClientFactory;
using apollo::drivers::canbus::CanClient;
using apollo::drivers::canbus::CanReceiver;
using apollo::drivers::canbus::SenderMessage;
using apollo::drivers::canbus::SensorCanbusConf;

class UltrasonicRadarCanbus : public apollo::common::ApolloApp {
 public:
  // TODO(lizh): check whether we need a new msg item, say
  // MonitorMessageItem::SENSORCANBUS
  UltrasonicRadarCanbus()
      : monitor_logger_(apollo::common::monitor::MonitorMessageItem::CANBUS) {}

  /**
  * @brief obtain module name
  * @return module name
  */
  std::string Name() const override;

  /**
  * @brief module initialization function
  * @return initialization status
  */
  apollo::common::Status Init() override;

  /**
  * @brief module start function
  * @return start status
  */
  apollo::common::Status Start() override;

  /**
  * @brief module stop function
  */
  void Stop() override;

 private:
  void PublishSensorData();
  Status OnError(const std::string &error_msg);
  void RegisterCanClients();

  UltrasonicRadarConf ultrasonic_radar_conf_;
  std::shared_ptr<CanClient> can_client_;
  CanReceiver<Ultrasonic> can_receiver_;
  std::unique_ptr<UltrasonicRadarMessageManager> sensor_message_manager_;

  int64_t last_timestamp_ = 0;
  apollo::common::monitor::MonitorLogger monitor_logger_;
};

}  // namespace ultrasonic_radar
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_RADAR_ULTRASONIC_RADAR_ULTRASONIC_RADAR_CANBUS_H_
