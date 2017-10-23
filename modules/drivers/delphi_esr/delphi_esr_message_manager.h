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
 * @file delphi_esr_message_manager.h
 * @brief The class of DelphiESRMessageManager
 */
#ifndef MODULES_DRIVERS_DELPHI_ESR_DELPHI_ESR_MESSAGE_MANAGER_H_
#define MODULES_DRIVERS_DELPHI_ESR_DELPHI_ESR_MESSAGE_MANAGER_H_

#include "modules/drivers/canbus/can_comm/message_manager.h"
#include "modules/drivers/proto/delphi_esr.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/drivers/canbus/sensor_gflags.h"

#include "modules/drivers/delphi_esr/protocol/acm_inst_req_7e0.h"
#include "modules/drivers/delphi_esr/protocol/acm_inst_resp_7e4.h"
#include "modules/drivers/delphi_esr/protocol/esr_sim1_5c0.h"
#include "modules/drivers/delphi_esr/protocol/esr_status1_4e0.h"
#include "modules/drivers/delphi_esr/protocol/esr_status2_4e1.h"
#include "modules/drivers/delphi_esr/protocol/esr_status3_4e2.h"
#include "modules/drivers/delphi_esr/protocol/esr_status4_4e3.h"
#include "modules/drivers/delphi_esr/protocol/esr_status5_5e4.h"
#include "modules/drivers/delphi_esr/protocol/esr_status6_5e5.h"
#include "modules/drivers/delphi_esr/protocol/esr_status7_5e6.h"
#include "modules/drivers/delphi_esr/protocol/esr_status8_5e7.h"
#include "modules/drivers/delphi_esr/protocol/esr_status9_5e8.h"
#include "modules/drivers/delphi_esr/protocol/esr_track01_500.h"
#include "modules/drivers/delphi_esr/protocol/esr_trackmotionpower_540.h"
#include "modules/drivers/delphi_esr/protocol/esr_valid1_5d0.h"
#include "modules/drivers/delphi_esr/protocol/esr_valid2_5d1.h"
#include "modules/drivers/delphi_esr/protocol/vehicle1_4f0.h"
#include "modules/drivers/delphi_esr/protocol/vehicle2_4f1.h"
#include "modules/drivers/delphi_esr/protocol/vehicle3_5f2.h"
#include "modules/drivers/delphi_esr/protocol/vehicle4_5f3.h"
#include "modules/drivers/delphi_esr/protocol/vehicle5_5f4.h"
#include "modules/drivers/delphi_esr/protocol/vehicle6_5f5.h"

namespace apollo {
namespace drivers {
namespace canbus {

using ::apollo::common::adapter::AdapterManager;

template <>
MessageManager<DelphiESR>::MessageManager() {
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Acminstreq7e0, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Acminstresp7e4, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrsim15c0, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrstatus14e0, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrstatus24e1, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrstatus34e2, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrstatus44e3, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrstatus55e4, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrstatus65e5, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrstatus75e6, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrstatus85e7, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrstatus95e8, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrack01500, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrtrackmotionpower540,
                      true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrvalid15d0, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Esrvalid25d1, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Vehicle14f0, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Vehicle24f1, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Vehicle35f2, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Vehicle45f3, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Vehicle55f4, true>();
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Vehicle65f5, true>();
}

template <>
ProtocolData<DelphiESR> *MessageManager<DelphiESR>::GetMutableProtocolDataById(
    const uint32_t message_id) {
  uint32_t converted_message_id = message_id;
  if (message_id >= 0x500 && message_id <= 0x539) {
    // repeated obstacle info
    converted_message_id = 0x500;
  }

  if (protocol_data_map_.find(converted_message_id) ==
      protocol_data_map_.end()) {
    ADEBUG << "Unable to get protocol data because of invalid message_id:"
           << message_id;
    return nullptr;
  }
  return protocol_data_map_[converted_message_id];
}

template <>
void MessageManager<DelphiESR>::Parse(const uint32_t message_id,
                                      const uint8_t *data, int32_t length) {
  ProtocolData<DelphiESR> *sensor_protocol_data =
      GetMutableProtocolDataById(message_id);
  if (sensor_protocol_data == nullptr) {
    return;
  }

  std::lock_guard<std::mutex> lock(sensor_data_mutex_);
  sensor_protocol_data->Parse(data, length, &sensor_data_);

  // trigger publishment
  if (message_id == 0x5E5) {
    ADEBUG << sensor_data_.ShortDebugString();

    AdapterManager::FillDelphiESRHeader(FLAGS_sensor_node_name, &sensor_data_);
    AdapterManager::PublishDelphiESR(sensor_data_);

    sensor_data_.Clear();
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

}  // namespace canbus
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_DELPHI_ESR_DELPHI_ESR_MESSAGE_MANAGER_H_
