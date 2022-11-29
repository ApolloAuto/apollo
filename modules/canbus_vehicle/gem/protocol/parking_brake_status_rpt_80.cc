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

#include "modules/canbus_vehicle/gem/protocol/parking_brake_status_rpt_80.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace gem {

using ::apollo::drivers::canbus::Byte;

Parkingbrakestatusrpt80::Parkingbrakestatusrpt80() {}
const int32_t Parkingbrakestatusrpt80::ID = 0x80;

void Parkingbrakestatusrpt80::Parse(const std::uint8_t* bytes, int32_t length,
                                    Gem* chassis) const {
  chassis->mutable_parking_brake_status_rpt_80()->set_parking_brake_enabled(
      parking_brake_enabled(bytes, length));
}

// config detail: {'name': 'parking_brake_enabled', 'enum': {0:
// 'PARKING_BRAKE_ENABLED_OFF', 1: 'PARKING_BRAKE_ENABLED_ON'},
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Parking_brake_status_rpt_80::Parking_brake_enabledType
Parkingbrakestatusrpt80::parking_brake_enabled(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  Parking_brake_status_rpt_80::Parking_brake_enabledType ret =
      static_cast<Parking_brake_status_rpt_80::Parking_brake_enabledType>(x);
  return ret;
}
}  // namespace gem
}  // namespace canbus
}  // namespace apollo
