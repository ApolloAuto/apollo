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

#include "modules/drivers/mobileye/protocol/aftermarket_669.h"
#include "modules/drivers/mobileye/protocol/details_737.h"
#include "modules/drivers/mobileye/protocol/details_738.h"
#include "modules/drivers/mobileye/protocol/details_739.h"
#include "modules/drivers/mobileye/protocol/details_73a.h"
#include "modules/drivers/mobileye/protocol/details_73b.h"
#include "modules/drivers/mobileye/protocol/lka_766.h"
#include "modules/drivers/mobileye/protocol/lka_767.h"
#include "modules/drivers/mobileye/protocol/lka_768.h"
#include "modules/drivers/mobileye/protocol/lka_769.h"
#include "modules/drivers/mobileye/protocol/next_76c.h"
#include "modules/drivers/mobileye/protocol/next_76d.h"
#include "modules/drivers/mobileye/protocol/num_76b.h"
#include "modules/drivers/mobileye/protocol/reference_76a.h"
#include "modules/drivers/proto/mobileye.pb.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"

namespace apollo {
namespace drivers {
namespace canbus {

template <>
MessageManager<Mobileye>::MessageManager() {
  AddRecvProtocolData<::apollo::drivers::mobileye::Aftermarket669, true>();
  AddRecvProtocolData<::apollo::drivers::mobileye::Details737, true>();
  AddRecvProtocolData<::apollo::drivers::mobileye::Details738, true>();
  AddRecvProtocolData<::apollo::drivers::mobileye::Details739, true>();
  AddRecvProtocolData<::apollo::drivers::mobileye::Details73a, true>();
  AddRecvProtocolData<::apollo::drivers::mobileye::Details73b, true>();
  AddRecvProtocolData<::apollo::drivers::mobileye::Lka766, true>();
  AddRecvProtocolData<::apollo::drivers::mobileye::Lka767, true>();
  AddRecvProtocolData<::apollo::drivers::mobileye::Lka768, true>();
  AddRecvProtocolData<::apollo::drivers::mobileye::Lka769, true>();
  AddRecvProtocolData<::apollo::drivers::mobileye::Next76c, true>();
  AddRecvProtocolData<::apollo::drivers::mobileye::Next76d, true>();
  AddRecvProtocolData<::apollo::drivers::mobileye::Num76b, true>();
  AddRecvProtocolData<::apollo::drivers::mobileye::Reference76a, true>();
}

template <>
ProtocolData<Mobileye>
    *MessageManager<Mobileye>::GetMutableProtocolDataById(
        const uint32_t message_id) {
  uint32_t converted_message_id = message_id;
  if (message_id >= 0x739 && message_id < (0x739 + 10 * 3)) {
    converted_message_id = 0x739 + ((message_id - 0x739) % 3);
  } else if (message_id >= 0x76c && message_id <= (0x76d + 7 * 2)) {
    converted_message_id = 0x76c + ((message_id - 0x76c) % 2);
  }

  if (protocol_data_map_.find(converted_message_id) ==
      protocol_data_map_.end()) {
    ADEBUG << "Unable to get protocol data because of invalid message_id:"
           << message_id;
    return nullptr;
  }
  return protocol_data_map_[converted_message_id];
}

}  // namespace canbus
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_MOBILEYE_MESSAGE_MANAGER_H_
