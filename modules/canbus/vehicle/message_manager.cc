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

#include "modules/canbus/vehicle/message_manager.h"

#include <thread>

#include "modules/canbus/common/byte.h"
#include "modules/common/log.h"
#include "modules/common/time/time.h"

namespace apollo {
namespace canbus {

using Clock = ::apollo::common::time::Clock;
using micros = std::chrono::microseconds;
using ::apollo::common::ErrorCode;

ProtocolData* MessageManager::GetMutableProtocolDataById(
    const uint32_t message_id) {
  if (protocol_data_map_.find(message_id) == protocol_data_map_.end()) {
    ADEBUG << "Unable to get protocol data because of invalid message_id:"
           << message_id;
    return nullptr;
  }
  return protocol_data_map_[message_id];
}

void MessageManager::Parse(const uint32_t message_id, const uint8_t* data,
                           int32_t length, struct timeval timestamp) {
  ProtocolData* protocol_data = GetMutableProtocolDataById(message_id);
  if (protocol_data == nullptr) {
    return;
  }
  if (timestamp.tv_sec == static_cast<time_t>(0)) {
    std::lock_guard<std::mutex> lock(chassis_detail_mutex_);
    protocol_data->Parse(data, length, &chassis_detail_);
  } else {
    // TODO: only lincoln implemented this virtual function
    std::lock_guard<std::mutex> lock(chassis_detail_mutex_);
    protocol_data->Parse(data, length, timestamp, &chassis_detail_);
  }
  received_ids_.insert(message_id);
  // check if need to check period
  const auto it = check_ids_.find(message_id);
  if (it != check_ids_.end()) {
    const int64_t time = apollo::common::time::AsInt64<micros>(Clock::Now());
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

ErrorCode MessageManager::GetChassisDetail(
    ChassisDetail* const chassis_detail) {
  if (chassis_detail == nullptr) {
    AERROR << "Failed to get chassis_detail due to nullptr.";
    return ErrorCode::CANBUS_ERROR;
  }
  std::lock_guard<std::mutex> lock(chassis_detail_mutex_);
  chassis_detail->CopyFrom(chassis_detail_);
  return ErrorCode::OK;
}

void MessageManager::ResetSendMessages() {
  for (auto& protocol_data : send_protocol_data_) {
    if (protocol_data == nullptr) {
      AERROR << "Invalid protocol data.";
    } else {
      protocol_data->Reset();
    }
  }
}

}  // namespace canbus
}  // namespace apollo
