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

#include "modules/canbus/vehicle/minibus/protocol/bus_vehicle_status_18fa0517.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace minibus {

using ::apollo::drivers::canbus::Byte;

Busvehiclestatus18fa0517::Busvehiclestatus18fa0517() {}
const int32_t Busvehiclestatus18fa0517::ID = 0x38fa0517;

void Busvehiclestatus18fa0517::Parse(const std::uint8_t* bytes, int32_t length,
                                     ChassisDetail* chassis) const {
  uint8_t statsu_id = bus_vehicle_sta_id(bytes, length);
  switch (statsu_id) {
    case 1:
      chassis->mutable_minibus()
          ->mutable_bus_vehicle_status_18fa0517()
          ->set_busv_aircondition_status(
              busv_aircondition_status(bytes, length));
      chassis->mutable_minibus()
          ->mutable_bus_vehicle_status_18fa0517()
          ->set_busv_smalllamp_status(busv_smalllamp_status(bytes, length));
      chassis->mutable_minibus()
          ->mutable_bus_vehicle_status_18fa0517()
          ->set_busv_turnleft_stauts(busv_turnleft_stauts(bytes, length));
      chassis->mutable_minibus()
          ->mutable_bus_vehicle_status_18fa0517()
          ->set_busv_turnright_status(busv_turnright_status(bytes, length));
      chassis->mutable_minibus()
          ->mutable_bus_vehicle_status_18fa0517()
          ->set_busv_lowbeam_status(busv_lowbeam_status(bytes, length));
      chassis->mutable_minibus()
          ->mutable_bus_vehicle_status_18fa0517()
          ->set_busv_horn_status(busv_horn_status(bytes, length));
      chassis->mutable_minibus()
          ->mutable_bus_vehicle_status_18fa0517()
          ->set_busv_park_status(busv_park_status(bytes, length));
      chassis->mutable_minibus()
          ->mutable_bus_vehicle_status_18fa0517()
          ->set_busv_break_status(busv_break_status(bytes, length));
      chassis->mutable_minibus()
          ->mutable_bus_vehicle_status_18fa0517()
          ->set_busv_fdoor_status(busv_fdoor_status(bytes, length));
      chassis->mutable_minibus()
          ->mutable_bus_vehicle_status_18fa0517()
          ->set_busv_reversing_status(busv_reversing_status(bytes, length));
      chassis->mutable_minibus()
          ->mutable_bus_vehicle_status_18fa0517()
          ->set_bus_vehicle_sta_id(bus_vehicle_sta_id(bytes, length));
      break;
    case 2:
      chassis->mutable_minibus()
          ->mutable_bus_vehicle_status_18fa0517()
          ->set_busv_emergencyhammer_worning(
              busv_emergencyhammer_worning(bytes, length));
      chassis->mutable_minibus()
          ->mutable_bus_vehicle_status_18fa0517()
          ->set_busv_body_notify(busv_body_notify(bytes, length));
      chassis->mutable_minibus()
          ->mutable_bus_vehicle_status_18fa0517()
          ->set_busv_break_wear(busv_break_wear(bytes, length));
      break;

    default:
      break;
  }
  // chassis->mutable_minibus()
  //     ->mutable_bus_vehicle_status_18fa0517()
  //     ->set_busv_emergencyhammer_worning(
  //         busv_emergencyhammer_worning(bytes, length));
  // chassis->mutable_minibus()
  //     ->mutable_bus_vehicle_status_18fa0517()
  //     ->set_busv_break_wear(busv_break_wear(bytes, length));
  // chassis->mutable_minibus()
  //     ->mutable_bus_vehicle_status_18fa0517()
  //     ->set_busv_body_notify(busv_body_notify(bytes, length));
}

// config detail: {'bit': 40, 'is_signed_var': False, 'len': 2, 'name':
// 'busv_emergencyhammer_worning', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'int'}
int Busvehiclestatus18fa0517::busv_emergencyhammer_worning(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 48, 'is_signed_var': False, 'len': 2, 'name':
// 'busv_break_wear', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Busvehiclestatus18fa0517::busv_break_wear(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 44, 'is_signed_var': False, 'len': 2, 'name':
// 'busv_body_notify', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Busvehiclestatus18fa0517::busv_body_notify(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(4, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 48, 'is_signed_var': False, 'len': 2, 'name':
// 'busv_aircondition_status', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'int'}
int Busvehiclestatus18fa0517::busv_aircondition_status(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 40, 'is_signed_var': False, 'len': 2, 'name':
// 'busv_smalllamp_status', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Busvehiclestatus18fa0517::busv_smalllamp_status(const std::uint8_t* bytes,
                                                    int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 36, 'enum': {0: 'BUSV_TURNLEFT_STAUTS_OFF', 1:
// 'BUSV_TURNLEFT_STAUTS_ON'}, 'is_signed_var': False, 'len': 2, 'name':
// 'busv_turnleft_stauts', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Bus_vehicle_status_18fa0517::Busv_turnleft_stautsType
Busvehiclestatus18fa0517::busv_turnleft_stauts(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(4, 2);

  Bus_vehicle_status_18fa0517::Busv_turnleft_stautsType ret =
      static_cast<Bus_vehicle_status_18fa0517::Busv_turnleft_stautsType>(x);
  return ret;
}

// config detail: {'bit': 34, 'enum': {0: 'BUSV_TURNRIGHT_STATUS_OFF', 1:
// 'BUSV_TURNRIGHT_STATUS_ON'}, 'is_signed_var': False, 'len': 2, 'name':
// 'busv_turnright_status', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Bus_vehicle_status_18fa0517::Busv_turnright_statusType
Busvehiclestatus18fa0517::busv_turnright_status(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(2, 2);

  Bus_vehicle_status_18fa0517::Busv_turnright_statusType ret =
      static_cast<Bus_vehicle_status_18fa0517::Busv_turnright_statusType>(x);
  return ret;
}

// config detail: {'bit': 32, 'is_signed_var': False, 'len': 2, 'name':
// 'busv_lowbeam_status', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Busvehiclestatus18fa0517::busv_lowbeam_status(const std::uint8_t* bytes,
                                                  int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 22, 'enum': {0: 'BUSV_HORN_STATUS_OFF', 1:
// 'BUSV_HORN_STATUS_ON'}, 'is_signed_var': False, 'len': 2, 'name':
// 'busv_horn_status', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Bus_vehicle_status_18fa0517::Busv_horn_statusType
Busvehiclestatus18fa0517::busv_horn_status(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(6, 2);

  Bus_vehicle_status_18fa0517::Busv_horn_statusType ret =
      static_cast<Bus_vehicle_status_18fa0517::Busv_horn_statusType>(x);
  return ret;
}

// config detail: {'bit': 18, 'is_signed_var': False, 'len': 2, 'name':
// 'busv_park_status', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Busvehiclestatus18fa0517::busv_park_status(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(2, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 2, 'name':
// 'busv_break_status', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Busvehiclestatus18fa0517::busv_break_status(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 10, 'enum': {0: 'BUSV_FDOOR_STATUS_DOOR_ACTION', 1:
// 'BUSV_FDOOR_STATUS_DOOR_OPENED', 2: 'BUSV_FDOOR_STATUS_DOOR_CLOSED'},
// 'is_signed_var': False, 'len': 2, 'name': 'busv_fdoor_status', 'offset': 0.0,
// 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '',
// 'precision': 1.0, 'type': 'enum'}
Bus_vehicle_status_18fa0517::Busv_fdoor_statusType
Busvehiclestatus18fa0517::busv_fdoor_status(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(2, 2);

  Bus_vehicle_status_18fa0517::Busv_fdoor_statusType ret =
      static_cast<Bus_vehicle_status_18fa0517::Busv_fdoor_statusType>(x);
  return ret;
}

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 2, 'name':
// 'busv_reversing_status', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Busvehiclestatus18fa0517::busv_reversing_status(const std::uint8_t* bytes,
                                                    int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name':
// 'bus_vehicle_sta_id', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Busvehiclestatus18fa0517::bus_vehicle_sta_id(const std::uint8_t* bytes,
                                                 int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
