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
 * @file sensor_message_manager.h
 * @brief The class of SensorMessageManager
 */
#ifndef MODULES_DRIVERS_MOBILEYE_MESSAGE_MANAGER_H_
#define MODULES_DRIVERS_MOBILEYE_MESSAGE_MANAGER_H_

#include "modules/drivers/mobileye/protocol/details_738.h"
#include "modules/drivers/mobileye/protocol/details_739.h"
#include "modules/drivers/mobileye/protocol/details_73a.h"
#include "modules/drivers/mobileye/protocol/details_73b.h"
#include "modules/drivers/proto/mobileye.pb.h"
#include "modules/drivers/sensor_message_manager.h"

namespace apollo {
namespace drivers {

template <>
SensorMessageManager<Mobileye>::SensorMessageManager() {
  AddRecvProtocolData<::apollo::drivers::mobileye::Details738, true>();
  AddRecvProtocolData<::apollo::drivers::mobileye::Details739, true>();
  AddRecvProtocolData<::apollo::drivers::mobileye::Details73a, true>();
  AddRecvProtocolData<::apollo::drivers::mobileye::Details73b, true>();
}

template <>
SensorProtocolData<Mobileye>
    *SensorMessageManager<Mobileye>::GetMutableSensorProtocolDataById(
        const uint32_t message_id) {
  uint32_t converted_message_id = message_id;
  if (message_id >= 0x739 && message_id < (0x739 + 10 * 3)) {
    converted_message_id = 0x739 + ((message_id - 0x739) % 3);
  } else if (message_id >= 0x76c && message_id <= (0x76d + 7 * 2)) {
    converted_message_id = 0x76c + ((message_id - 0x76c) % 2);
  }

  if (sensor_protocol_data_map_.find(converted_message_id) ==
      sensor_protocol_data_map_.end()) {
    ADEBUG << "Unable to get protocol data because of invalid message_id:"
           << message_id;
    return nullptr;
  }
  return sensor_protocol_data_map_[converted_message_id];
}

}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_MOBILEYE_MESSAGE_MANAGER_H_
