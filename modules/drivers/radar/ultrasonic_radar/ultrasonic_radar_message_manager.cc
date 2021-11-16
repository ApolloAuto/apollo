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

#include "modules/drivers/radar/ultrasonic_radar/ultrasonic_radar_message_manager.h"

#include "modules/common/util/message_util.h"

namespace apollo {
namespace drivers {
namespace ultrasonic_radar {

UltrasonicRadarMessageManager::UltrasonicRadarMessageManager(
    const int entrance_num,
    const std::shared_ptr<::apollo::cyber::Writer<Ultrasonic>> &writer)
    : entrance_num_(entrance_num), ultrasonic_radar_writer_(writer) {
  sensor_data_.mutable_ranges()->Resize(entrance_num_, 0.0);
}

void UltrasonicRadarMessageManager::set_can_client(
    std::shared_ptr<CanClient> can_client) {
  can_client_ = can_client;
}

void UltrasonicRadarMessageManager::Parse(const uint32_t message_id,
                                          const uint8_t *data, int32_t length) {
  if (message_id == 0x301) {
    sensor_data_.set_ranges(0, data[1]);
    sensor_data_.set_ranges(1, data[2]);
    sensor_data_.set_ranges(2, data[3]);
    sensor_data_.set_ranges(3, data[4]);
  } else if (message_id == 0x302) {
    sensor_data_.set_ranges(4, data[1]);
    sensor_data_.set_ranges(5, data[2]);
    sensor_data_.set_ranges(6, data[3]);
    sensor_data_.set_ranges(7, data[4]);
  } else if (message_id == 0x303) {
    sensor_data_.set_ranges(8, data[1]);
    sensor_data_.set_ranges(9, data[2]);
  } else if (message_id == 0x304) {
    sensor_data_.set_ranges(10, data[1]);
    sensor_data_.set_ranges(11, data[2]);
    common::util::FillHeader("ultrasonic_radar", &sensor_data_);
    ultrasonic_radar_writer_->Write(sensor_data_);
  }

  received_ids_.insert(message_id);
  // check if need to check period
  const auto it = check_ids_.find(message_id);
  if (it != check_ids_.end()) {
    const int64_t time = Time::Now().ToMicrosecond();
    it->second.real_period = time - it->second.last_time;
    // if period 1.5 large than base period, inc error_count
    const double period_multiplier = 1.5;
    if (it->second.real_period >
        (static_cast<double>(it->second.period) * period_multiplier)) {
      it->second.error_count += 1;
    } else {
      it->second.error_count = 0;
    }
    it->second.last_time = time;
  }
}

}  // namespace ultrasonic_radar
}  // namespace drivers
}  // namespace apollo
