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

#include "modules/canbus/vehicle/transit/protocol/llc_vehiclelimits_24.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace transit {

using ::apollo::drivers::canbus::Byte;

Llcvehiclelimits24::Llcvehiclelimits24() {}
const int32_t Llcvehiclelimits24::ID = 0x24;

void Llcvehiclelimits24::Parse(const std::uint8_t* bytes, int32_t length,
                               ChassisDetail* chassis) const {
  chassis->mutable_transit()
      ->mutable_llc_vehiclelimits_24()
      ->set_llc_fbk_maxsteeringangle(llc_fbk_maxsteeringangle(bytes, length));
  chassis->mutable_transit()
      ->mutable_llc_vehiclelimits_24()
      ->set_llc_fbk_maxbrakepercent(llc_fbk_maxbrakepercent(bytes, length));
}

// config detail: {'description': 'Steering angle feedback', 'offset': 0.0,
// 'precision': 1.0, 'len': 12, 'name': 'llc_fbk_maxsteeringangle',
// 'is_signed_var': False, 'physical_range': '[0|4095]', 'bit': 12, 'type':
// 'int', 'order': 'intel', 'physical_unit': 'deg'}
int Llcvehiclelimits24::llc_fbk_maxsteeringangle(const std::uint8_t* bytes,
                                                 int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'description': 'Front brake pressure feedback', 'offset':
// 0.0, 'precision': 1.0, 'len': 12, 'name': 'llc_fbk_maxbrakepercent',
// 'is_signed_var': False, 'physical_range': '[0|113.8132]', 'bit': 0, 'type':
// 'double', 'order': 'intel', 'physical_unit': '%'}
double Llcvehiclelimits24::llc_fbk_maxbrakepercent(const std::uint8_t* bytes,
                                                   int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x;
  return ret;
}
}  // namespace transit
}  // namespace canbus
}  // namespace apollo
