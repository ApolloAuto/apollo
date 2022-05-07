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

#pragma once

#include "modules/canbus/proto/chassis_detail.pb.h"

#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace minibus {

class Busvehiclestatus18fa0517 : public ::apollo::drivers::canbus::ProtocolData<
                                     ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Busvehiclestatus18fa0517();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'bit': 40, 'is_signed_var': False, 'len': 2, 'name':
  // 'BUSV_EmergencyHammer_Worning', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  int busv_emergencyhammer_worning(const std::uint8_t* bytes,
                                   const int32_t length) const;

  // config detail: {'bit': 48, 'is_signed_var': False, 'len': 2, 'name':
  // 'BUSV_Break_Wear', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int busv_break_wear(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 44, 'is_signed_var': False, 'len': 2, 'name':
  // 'BUSV_Body_Notify', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int busv_body_notify(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 48, 'is_signed_var': False, 'len': 2, 'name':
  // 'BUSV_Aircondition_Status', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  int busv_aircondition_status(const std::uint8_t* bytes,
                               const int32_t length) const;

  // config detail: {'bit': 40, 'is_signed_var': False, 'len': 2, 'name':
  // 'BUSV_SmallLamp_Status', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int busv_smalllamp_status(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'bit': 36, 'enum': {0: 'BUSV_TURNLEFT_STAUTS_OFF', 1:
  // 'BUSV_TURNLEFT_STAUTS_ON'}, 'is_signed_var': False, 'len': 2, 'name':
  // 'BUSV_TurnLeft_Stauts', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Bus_vehicle_status_18fa0517::Busv_turnleft_stautsType busv_turnleft_stauts(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 34, 'enum': {0: 'BUSV_TURNRIGHT_STATUS_OFF', 1:
  // 'BUSV_TURNRIGHT_STATUS_ON'}, 'is_signed_var': False, 'len': 2, 'name':
  // 'BUSV_TurnRight_Status', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Bus_vehicle_status_18fa0517::Busv_turnright_statusType busv_turnright_status(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 2, 'name':
  // 'BUSV_LowBeam_Status', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int busv_lowbeam_status(const std::uint8_t* bytes,
                          const int32_t length) const;

  // config detail: {'bit': 22, 'enum': {0: 'BUSV_HORN_STATUS_OFF', 1:
  // 'BUSV_HORN_STATUS_ON'}, 'is_signed_var': False, 'len': 2, 'name':
  // 'BUSV_Horn_Status', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Bus_vehicle_status_18fa0517::Busv_horn_statusType busv_horn_status(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 18, 'is_signed_var': False, 'len': 2, 'name':
  // 'BUSV_Park_Status', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int busv_park_status(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 2, 'name':
  // 'BUSV_Break_Status', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int busv_break_status(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 10, 'enum': {0: 'BUSV_FDOOR_STATUS_DOOR_ACTION', 1:
  // 'BUSV_FDOOR_STATUS_DOOR_OPENED', 2: 'BUSV_FDOOR_STATUS_DOOR_CLOSED'},
  // 'is_signed_var': False, 'len': 2, 'name': 'BUSV_Fdoor_Status', 'offset':
  // 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '',
  // 'precision': 1.0, 'type': 'enum'}
  Bus_vehicle_status_18fa0517::Busv_fdoor_statusType busv_fdoor_status(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 2, 'name':
  // 'BUSV_Reversing_Status', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int busv_reversing_status(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name':
  // 'BUS_Vehicle_Sta_ID', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int bus_vehicle_sta_id(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
