/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/vehicle/hunter2/protocol/bms_data_feedback_361.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace hunter2 {

using ::apollo::drivers::canbus::Byte;

Bmsdatafeedback361::Bmsdatafeedback361() {}
const int32_t Bmsdatafeedback361::ID = 0x361;

void Bmsdatafeedback361::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_hunter2()->mutable_bms_data_feedback_361()->set_bms_battery_temperature(bms_battery_temperature(bytes, length));
  chassis->mutable_hunter2()->mutable_bms_data_feedback_361()->set_bms_battery_current(bms_battery_current(bytes, length));
  chassis->mutable_hunter2()->mutable_bms_data_feedback_361()->set_bms_battery_voltage(bms_battery_voltage(bytes, length));
  chassis->mutable_hunter2()->mutable_bms_data_feedback_361()->set_bms_battery_soh(bms_battery_soh(bytes, length));
  chassis->mutable_hunter2()->mutable_bms_data_feedback_361()->set_bms_battery_soc(bms_battery_soc(bytes, length));
}

// config detail: {'bit': 55, 'is_signed_var': False, 'len': 16, 'name': 'bms_battery_temperature', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|6553.5]', 'physical_unit': '', 'precision': 0.1, 'type': 'double'}
double Bmsdatafeedback361::bms_battery_temperature(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 7);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 39, 'is_signed_var': False, 'len': 16, 'name': 'bms_battery_current', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|6553.5]', 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
double Bmsdatafeedback361::bms_battery_current(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 23, 'is_signed_var': False, 'len': 16, 'name': 'bms_battery_voltage', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|655.35]', 'physical_unit': 'V', 'precision': 0.01, 'type': 'double'}
double Bmsdatafeedback361::bms_battery_voltage(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'bit': 15, 'is_signed_var': False, 'len': 8, 'name': 'bms_battery_soh', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Bmsdatafeedback361::bms_battery_soh(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 7, 'is_signed_var': False, 'len': 8, 'name': 'bms_battery_soc', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Bmsdatafeedback361::bms_battery_soc(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace hunter2
}  // namespace canbus
}  // namespace apollo
