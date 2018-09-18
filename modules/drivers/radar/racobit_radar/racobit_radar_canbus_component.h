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

#ifndef MODULES_DRIVERS_RADAR_RACOBIT_RADAR_RACOBIT_RADAR_CANBUS_COMPONENT_H_
#define MODULES_DRIVERS_RADAR_RACOBIT_RADAR_RACOBIT_RADAR_CANBUS_COMPONENT_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "cybertron/cybertron.h"

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
#include "modules/drivers/proto/racobit_radar.pb.h"
#include "modules/drivers/radar/racobit_radar/protocol/radar_config_200.h"
#include "modules/drivers/radar/racobit_radar/racobit_radar_message_manager.h"

/**
 * @namespace apollo::drivers
 * @brief apollo::drivers
 */
namespace apollo {
namespace drivers {
namespace racobit_radar {

/**
 * @class RacobitRadarCanbus
 *
 * @brief template of canbus-based sensor module main class (e.g.,
 * racobit_radar).
 */

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::monitor::MonitorMessageItem;
using apollo::common::time::Clock;
using apollo::drivers::canbus::CanClient;
using apollo::drivers::canbus::CanClientFactory;
using apollo::drivers::canbus::CanReceiver;
using apollo::drivers::canbus::SenderMessage;
using apollo::drivers::canbus::SensorCanbusConf;

class RacobitRadarCanbus : public apollo::cybertron::Component<> {
 public:
  // TODO(lizh): check whether we need a new msg item, say
  // MonitorMessageItem::SENSORCANBUS
  RacobitRadarCanbus()
      : monitor_logger_(apollo::common::monitor::MonitorMessageItem::CANBUS) {}
  ~RacobitRadarCanbus();

  /**
   * @brief module initialization function
   * @return initialization success(true) or not(false).
   */
  bool Init() override;

 private:
  /**
   * @brief module start function
   * @return start status
   */
  apollo::common::Status Start() override;

  /**
   * @brief module stop function
   */
  void Stop() override;

  Status OnError(const std::string &error_msg);
  void RegisterCanClients();
  apollo::common::ErrorCode ConfigureRadar();

  std::shared_ptr<CanClient> can_client_;
  CanReceiver<RacobitRadar> can_receiver_;
  std::unique_ptr<RacobitRadarMessageManager> sensor_message_manager_;

  int64_t last_timestamp_ = 0;
  apollo::common::monitor::MonitorLogger monitor_logger_;

  bool start_success_ = false;
  // cybertron
  RacobitRadarConf racobit_radar_conf_;
  std::shared_ptr<cybertron::Writer<RacobitRadar>> racobit_radar_writer_;
};

CYBERTRON_REGISTER_COMPONENT(RacobitRadarCanbus)

}  // namespace racobit_radar
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_RADAR_RACOBIT_RADAR_RACOBIT_RADAR_CANBUS_COMPONENT_H_
