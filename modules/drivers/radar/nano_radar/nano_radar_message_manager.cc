/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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
 * @file nano_radar_message_manager.h
 * @brief The class of NanoRadarMessageManager
 */

#include "modules/drivers/radar/nano_radar/nano_radar_message_manager.h"

#include "modules/common/util/message_util.h"
#include "modules/drivers/radar/nano_radar/protocol/object_general_info_60b.h"
#include "modules/drivers/radar/nano_radar/protocol/object_list_status_60a.h"
#include "modules/drivers/radar/nano_radar/protocol/radar_state_201.h"
#include "modules/drivers/radar/nano_radar/protocol/region_state_402.h"
#include "modules/drivers/radar/nano_radar/protocol/software_version_700.h"

namespace apollo {
namespace drivers {
namespace nano_radar {

using Time = apollo::cyber::Time;
using micros = std::chrono::microseconds;
using apollo::cyber::Writer;
using apollo::drivers::canbus::CanClient;
using apollo::drivers::canbus::ProtocolData;
using apollo::drivers::canbus::SenderMessage;

NanoRadarMessageManager::NanoRadarMessageManager(
    const std::shared_ptr<Writer<NanoRadar>> &writer)
    : nano_radar_writer_(writer) {
  AddRecvProtocolData<RadarState201, true>();
  AddRecvProtocolData<CollisionDetectionRegionState402, true>();
  AddRecvProtocolData<ObjectGeneralInfo60B, true>();
  AddRecvProtocolData<ObjectListStatus60A, true>();
  AddRecvProtocolData<SoftwareVersion700, true>();
}

void NanoRadarMessageManager::set_radar_conf(RadarConf radar_conf) {
  radar_config_.set_radar_conf(radar_conf);
}

void NanoRadarMessageManager::set_can_client(
    std::shared_ptr<CanClient> can_client) {
  can_client_ = can_client;
}

ProtocolData<NanoRadar> *NanoRadarMessageManager::GetMutableProtocolDataById(
    const uint32_t message_id) {
  uint32_t converted_message_id = message_id;
  if (protocol_data_map_.find(converted_message_id) ==
      protocol_data_map_.end()) {
    ADEBUG << "Unable to get protocol data because of invalid message_id:"
           << message_id;
    return nullptr;
  }
  return protocol_data_map_[converted_message_id];
}

void NanoRadarMessageManager::Parse(const uint32_t message_id,
                                    const uint8_t *data, int32_t length) {
  ProtocolData<NanoRadar> *sensor_protocol_data =
      GetMutableProtocolDataById(message_id);
  if (sensor_protocol_data == nullptr) {
    return;
  }

  std::lock_guard<std::mutex> lock(sensor_data_mutex_);
  if (!is_configured_ && message_id != RadarState201::ID) {
    // read radar state message first
    return;
  }

  // trigger publishment
  if (message_id == ObjectListStatus60A::ID) {
    ADEBUG << sensor_data_.ShortDebugString();
    nano_radar_writer_->Write(sensor_data_);
    sensor_data_.Clear();
    // fill header when receive the general info message
    common::util::FillHeader("nano_radar", &sensor_data_);
  }

  sensor_protocol_data->Parse(data, length, &sensor_data_);

  if (message_id == RadarState201::ID) {
    ADEBUG << sensor_data_.ShortDebugString();
    if (sensor_data_.radar_state().send_quality() ==
            radar_config_.radar_conf().send_quality() &&
        sensor_data_.radar_state().send_ext_info() ==
            radar_config_.radar_conf().send_ext_info() &&
        sensor_data_.radar_state().max_distance() ==
            radar_config_.radar_conf().max_distance() &&
        sensor_data_.radar_state().output_type() ==
            radar_config_.radar_conf().output_type() &&
        sensor_data_.radar_state().rcs_threshold() ==
            radar_config_.radar_conf().rcs_threshold() &&
        sensor_data_.radar_state().radar_power() ==
            radar_config_.radar_conf().radar_power()) {
      is_configured_ = true;
    } else {
      AINFO << "configure radar again";
      SenderMessage<NanoRadar> sender_message200(RadarConfig200::ID,
                                                 &radar_config_);
      sender_message200.Update();
      can_client_->SendSingleFrame({sender_message200.CanFrame()});
    }
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

}  // namespace nano_radar
}  // namespace drivers
}  // namespace apollo
