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
namespace lexus {

class Componentrpt20 : public ::apollo::drivers::canbus::ProtocolData<
                           ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Componentrpt20();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'name': 'COMPONENT_TYPE', 'enum': {0:
  // 'COMPONENT_TYPE_PACMOD', 1: 'COMPONENT_TYPE_PACMINI', 2:
  // 'COMPONENT_TYPE_PACMICRO'}, 'precision': 1.0, 'len': 8, 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|255]', 'bit': 7, 'type':
  // 'enum', 'order': 'motorola', 'physical_unit': ''}
  Component_rpt_20::Component_typeType component_type(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'COMPONENT_FUNC', 'enum': {0:
  // 'COMPONENT_FUNC_PACMOD', 1: 'COMPONENT_FUNC_STEERING_AND_STEERING_COLUMN',
  // 2: 'COMPONENT_FUNC_ACCELERATOR_AND_BRAKING', 3: 'COMPONENT_FUNC_BRAKING',
  // 4: 'COMPONENT_FUNC_SHIFTING', 5: 'COMPONENT_FUNC_STEERING', 6:
  // 'COMPONENT_FUNC_E_SHIFTER', 7: 'COMPONENT_FUNC_WATCHDOG'},
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|255]', 'bit': 15, 'type': 'enum', 'order':
  // 'motorola', 'physical_unit': ''}
  Component_rpt_20::Component_funcType component_func(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'COUNTER', 'offset': 0.0, 'precision': 1.0, 'len':
  // 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 19, 'type':
  // 'int', 'order': 'motorola', 'physical_unit': ''}
  int counter(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'COMPLEMENT', 'offset': 0.0, 'precision': 1.0,
  // 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 23,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int complement(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CONFIG_FAULT', 'offset': 0.0, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 24,
  // 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool config_fault(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
