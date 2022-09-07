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

#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "cyber/cyber.h"

#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/util/util.h"
#include "modules/drivers/canbus/can_client/can_client.h"
#include "modules/drivers/canbus/can_comm/can_receiver.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"
#include "modules/common_msgs/drivers_msgs/can_card_parameter.pb.h"
#include "modules/common_msgs/sensor_msgs/conti_radar.pb.h"
#include "modules/drivers/radar/conti_radar/conti_radar_message_manager.h"
#include "modules/drivers/radar/conti_radar/protocol/motion_input_speed_300.h"
#include "modules/drivers/radar/conti_radar/protocol/motion_input_yawrate_301.h"
#include "modules/drivers/radar/conti_radar/protocol/radar_config_200.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"

/**
 * @namespace apollo::drivers
 * @brief apollo::drivers
 */
namespace apollo {
namespace drivers {
namespace conti_radar {

/**
 * @class ContiRadarCanbus
 *
 * @brief template of canbus-based sensor module main class (e.g., conti_radar).
 */

class ContiRadarCanbusComponent : public apollo::cyber::Component<> {
 public:
  ContiRadarCanbusComponent();
  ~ContiRadarCanbusComponent();
  bool Init() override;

 private:
  bool OnError(const std::string& error_msg);
  void RegisterCanClients();
  apollo::common::ErrorCode ConfigureRadar();
  bool Start();
  void Stop();

  ContiRadarConf conti_radar_conf_;
  std::shared_ptr<apollo::drivers::canbus::CanClient> can_client_;
  apollo::drivers::canbus::CanReceiver<ContiRadar> can_receiver_;
  std::unique_ptr<ContiRadarMessageManager> sensor_message_manager_;
  std::shared_ptr<apollo::cyber::Writer<ContiRadar>> conti_radar_writer_;
  std::shared_ptr<
      apollo::cyber::Reader<apollo::localization::LocalizationEstimate>>
      pose_reader_;
  void PoseCallback(
      const std::shared_ptr<apollo::localization::LocalizationEstimate>& pose);
  uint64_t last_nsec_ = 0;

  int64_t last_timestamp_ = 0;
  bool start_success_ = false;
  apollo::common::monitor::MonitorLogBuffer monitor_logger_buffer_;
};

CYBER_REGISTER_COMPONENT(ContiRadarCanbusComponent)

}  // namespace conti_radar
}  // namespace drivers
}  // namespace apollo
