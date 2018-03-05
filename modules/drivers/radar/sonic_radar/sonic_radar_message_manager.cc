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
 * @file sonic_radar_message_manager.h
 * @brief The class of SonicRadarMessageManager
 */

#include "modules/drivers/radar/sonic_radar/sonic_radar_message_manager.h"
#include "modules/drivers/radar/sonic_radar/protocol/range_fill_301.h"
#include "modules/drivers/radar/sonic_radar/protocol/range_fill_302.h"
#include "modules/drivers/radar/sonic_radar/protocol/range_fill_303.h"
#include "modules/drivers/radar/sonic_radar/protocol/range_fill_304.h"


namespace apollo {
namespace drivers {
namespace sonic_radar {

using ::apollo::common::adapter::AdapterManager;

SonicRadarMessageManager::SonicRadarMessageManager() {
  AddRecvProtocolData<RangeFill301, true>();
  AddRecvProtocolData<RangeFill302, true>();
  AddRecvProtocolData<RangeFill303, true>();
  AddRecvProtocolData<RangeFill304, true>();
}

void SonicRadarMessageManager::set_can_client(
    std::shared_ptr<CanClient> can_client) {
  can_client_ = can_client;
}

void SonicRadarMessageManager::Parse(const uint32_t message_id,
                                     const uint8_t *data, int32_t length) {
  ProtocolData<Ultrasonic> *sensor_protocol_data =
      GetMutableProtocolDataById(message_id);
  AINFO << "SonicRadarMessageManager::Parse, message_id: " << message_id
        << ", length: " << length;
  sensor_protocol_data->Parse(data, length, &sensor_data_);
  static std::vector<bool> data_set(4, false);
  if (message_id == 0x301) {
    if (data_set[0]) {
      AINFO << "one set again.";
    }
    data_set[0] = true;
  } else if (message_id == 0x302) { // for 4 - 7
    if (data_set[1]) {
      AINFO << "two set again.";
    }
    data_set[1] = true;
  } else if (message_id == 0x303) { // for 8 - 9
    if (data_set[2]) {
      AINFO << "three set again.";
    }
    data_set[2] = true;
  } else if (message_id == 0x304) {
    if (data_set[3]) {
      AINFO << "four set again.";
    }
    data_set[3] = true;
    for (int i = 0; i < 4; ++i) {
      if (!data_set[i]) {
        AINFO << "set not complete.";
      }
      data_set[i] = false;
    }
    sensor_data_.Clear();
    AdapterManager::FillUltrasonicHeader(FLAGS_sensor_node_name, &sensor_data_);
    //AdapterManager::PublishUltrasonic(sensor_data_);
  } else {
    ;
  }

  received_ids_.insert(message_id);
  // check if need to check period
  const auto it = check_ids_.find(message_id);
  if (it != check_ids_.end()) {
    const int64_t time = common::time::AsInt64<micros>(Clock::Now());
    it->second.real_period = time - it->second.last_time;
    // if period 1.5 large than base period, inc error_count
    const double period_multiplier = 1.5;
    if (it->second.real_period > (it->second.period * period_multiplier)) {
      it->second.error_count += 1;
    } else {
      it->second.error_count = 0;
    }
    it->second.last_time = time;
  }
}

}  // namespace sonic_radar
}  // namespace drivers
}  // namespace apollo
