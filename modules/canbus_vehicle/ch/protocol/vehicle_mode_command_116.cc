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

#include "modules/canbus_vehicle/ch/protocol/vehicle_mode_command_116.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace ch {

using ::apollo::drivers::canbus::Byte;

const int32_t Vehiclemodecommand116::ID = 0x116;

// public
Vehiclemodecommand116::Vehiclemodecommand116() { Reset(); }

uint32_t Vehiclemodecommand116::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 500 * 1000;
  return PERIOD;
}

void Vehiclemodecommand116::UpdateData(uint8_t* data) {
  set_p_vin_req_cmd(data, vin_req_cmd_);
}

void Vehiclemodecommand116::Reset() {
  // TODO(All) :  you should check this manually
  vin_req_cmd_ = Vehicle_mode_command_116::VIN_REQ_CMD_VIN_REQ_DISABLE;
}

Vehiclemodecommand116* Vehiclemodecommand116::set_vin_req_cmd(
    Vehicle_mode_command_116::Vin_req_cmdType vin_req_cmd) {
  vin_req_cmd_ = vin_req_cmd;
  return this;
}

// config detail: {'bit': 0, 'description': 'Request VIN(Command)', 'enum': {0:
// 'VIN_REQ_CMD_VIN_REQ_DISABLE', 1: 'VIN_REQ_CMD_VIN_REQ_ENABLE'},
// 'is_signed_var': False, 'len': 1, 'name': 'VIN_REQ_CMD', 'offset': 0.0,
// 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '',
// 'precision': 1.0, 'type': 'enum'}
void Vehiclemodecommand116::set_p_vin_req_cmd(
    uint8_t* data, Vehicle_mode_command_116::Vin_req_cmdType vin_req_cmd) {
  int x = vin_req_cmd;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 0, 1);
}

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
