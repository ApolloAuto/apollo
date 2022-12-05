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

#include "modules/canbus_vehicle/lexus/protocol/component_rpt_20.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace lexus {

using ::apollo::drivers::canbus::Byte;

Componentrpt20::Componentrpt20() {}
const int32_t Componentrpt20::ID = 0x20;

void Componentrpt20::Parse(const std::uint8_t* bytes, int32_t length,
                           Lexus* chassis) const {
  chassis->mutable_component_rpt_20()->set_component_type(
      component_type(bytes, length));
  chassis->mutable_component_rpt_20()->set_component_func(
      component_func(bytes, length));
  chassis->mutable_component_rpt_20()->set_counter(
      counter(bytes, length));
  chassis->mutable_component_rpt_20()->set_complement(
      complement(bytes, length));
  chassis->mutable_component_rpt_20()->set_config_fault(
      config_fault(bytes, length));
}

// config detail: {'name': 'component_type', 'enum': {0:
// 'COMPONENT_TYPE_PACMOD', 1: 'COMPONENT_TYPE_PACMINI', 2:
// 'COMPONENT_TYPE_PACMICRO'}, 'precision': 1.0, 'len': 8, 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|255]', 'bit': 7, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': ''}
Component_rpt_20::Component_typeType Componentrpt20::component_type(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Component_rpt_20::Component_typeType ret =
      static_cast<Component_rpt_20::Component_typeType>(x);
  return ret;
}

// config detail: {'name': 'component_func', 'enum': {0:
// 'COMPONENT_FUNC_PACMOD', 1: 'COMPONENT_FUNC_STEERING_AND_STEERING_COLUMN', 2:
// 'COMPONENT_FUNC_ACCELERATOR_AND_BRAKING', 3: 'COMPONENT_FUNC_BRAKING', 4:
// 'COMPONENT_FUNC_SHIFTING', 5: 'COMPONENT_FUNC_STEERING', 6:
// 'COMPONENT_FUNC_E_SHIFTER', 7: 'COMPONENT_FUNC_WATCHDOG'}, 'precision': 1.0,
// 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|255]',
// 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Component_rpt_20::Component_funcType Componentrpt20::component_func(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Component_rpt_20::Component_funcType ret =
      static_cast<Component_rpt_20::Component_funcType>(x);
  return ret;
}

// config detail: {'name': 'counter', 'offset': 0.0, 'precision': 1.0, 'len': 4,
// 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 19, 'type': 'int',
// 'order': 'motorola', 'physical_unit': ''}
int Componentrpt20::counter(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'complement', 'offset': 0.0, 'precision': 1.0, 'len':
// 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 23, 'type':
// 'int', 'order': 'motorola', 'physical_unit': ''}
int Componentrpt20::complement(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'config_fault', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 24,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Componentrpt20::config_fault(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}
}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
