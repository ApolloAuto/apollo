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

#include "modules/canbus/vehicle/hunter2/protocol/bms_status_feedback_362.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace hunter2 {

using ::apollo::drivers::canbus::Byte;

Bmsstatusfeedback362::Bmsstatusfeedback362() {}
const int32_t Bmsstatusfeedback362::ID = 0x362;

void Bmsstatusfeedback362::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_hunter2()->mutable_bms_status_feedback_362()->set_bms_warning_status_2(bms_warning_status_2(bytes, length));
  chassis->mutable_hunter2()->mutable_bms_status_feedback_362()->set_bms_warning_status_1(bms_warning_status_1(bytes, length));
  chassis->mutable_hunter2()->mutable_bms_status_feedback_362()->set_bms_alarm_status_2(bms_alarm_status_2(bytes, length));
  chassis->mutable_hunter2()->mutable_bms_status_feedback_362()->set_bms_alarm_status_1(bms_alarm_status_1(bytes, length));
}

// config detail: {'bit': 31, 'is_signed_var': False, 'len': 8, 'name': 'bms_warning_status_2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Bmsstatusfeedback362::bms_warning_status_2(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 23, 'is_signed_var': False, 'len': 8, 'name': 'bms_warning_status_1', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Bmsstatusfeedback362::bms_warning_status_1(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 15, 'is_signed_var': False, 'len': 8, 'name': 'bms_alarm_status_2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Bmsstatusfeedback362::bms_alarm_status_2(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 7, 'is_signed_var': False, 'len': 8, 'name': 'bms_alarm_status_1', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Bmsstatusfeedback362::bms_alarm_status_1(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace hunter2
}  // namespace canbus
}  // namespace apollo
