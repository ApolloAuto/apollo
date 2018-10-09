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

#pragma once

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace gem {

class Parkingbrakestatusrpt80 : public ::apollo::drivers::canbus::ProtocolData<
                                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Parkingbrakestatusrpt80();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'name': 'PARKING_BRAKE_ENABLED', 'enum': {0:
  // 'PARKING_BRAKE_ENABLED_OFF', 1: 'PARKING_BRAKE_ENABLED_ON'},
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Parking_brake_status_rpt_80::Parking_brake_enabledType parking_brake_enabled(
      const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace gem
}  // namespace canbus
}  // namespace apollo
