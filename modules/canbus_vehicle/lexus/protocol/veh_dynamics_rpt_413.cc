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

#include "modules/canbus_vehicle/lexus/protocol/veh_dynamics_rpt_413.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace lexus {

using ::apollo::drivers::canbus::Byte;

Vehdynamicsrpt413::Vehdynamicsrpt413() {}
const int32_t Vehdynamicsrpt413::ID = 0x413;

void Vehdynamicsrpt413::Parse(const std::uint8_t* bytes, int32_t length,
                              Lexus* chassis) const {
  chassis->mutable_veh_dynamics_rpt_413()->set_veh_g_forces(
      veh_g_forces(bytes, length));
}

// config detail: {'name': 'veh_g_forces', 'offset': 0.0, 'precision': 0.001,
// 'len': 8, 'is_signed_var': False, 'physical_range': '[-32.768|32.767]',
// 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
double Vehdynamicsrpt413::veh_g_forces(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.001000;
  return ret;
}
}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
