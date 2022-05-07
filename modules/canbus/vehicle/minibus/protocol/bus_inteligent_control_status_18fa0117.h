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

class Businteligentcontrolstatus18fa0117
    : public ::apollo::drivers::canbus::ProtocolData<
          ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Businteligentcontrolstatus18fa0117();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'bit': 28, 'is_signed_var': False, 'len': 2, 'name':
  // 'BUS_InnerDoor_DaySensor', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  int bus_innerdoor_daysensor(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'bit': 26, 'is_signed_var': False, 'len': 2, 'name':
  // 'BUS_OutDoor_DaySensor', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int bus_outdoor_daysensor(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'bit': 24, 'is_signed_var': False, 'len': 2, 'name':
  // 'BUS_OutDoor_key', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int bus_outdoor_key(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 22, 'is_signed_var': False, 'len': 2, 'name':
  // 'BUS_InnerDoor_Key', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int bus_innerdoor_key(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 20, 'is_signed_var': False, 'len': 2, 'name':
  // 'BUS_Breaking_Key', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int bus_breaking_key(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 18, 'is_signed_var': False, 'len': 2, 'name':
  // 'BUS_Reversing_key', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int bus_reversing_key(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 2, 'name':
  // 'BUS_Drive_key', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int bus_drive_key(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 14, 'is_signed_var': False, 'len': 2, 'name':
  // 'BUS_Joystick_Right_Key', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  int bus_joystick_right_key(const std::uint8_t* bytes,
                             const int32_t length) const;

  // config detail: {'bit': 12, 'is_signed_var': False, 'len': 2, 'name':
  // 'BUS_Joystick_Left_Key', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int bus_joystick_left_key(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'bit': 10, 'is_signed_var': False, 'len': 2, 'name':
  // 'BUS_Joystick_Behind_Key', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  int bus_joystick_behind_key(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 2, 'name':
  // 'BUS_Joystick_Front_Key', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  int bus_joystick_front_key(const std::uint8_t* bytes,
                             const int32_t length) const;

  // config detail: {'bit': 6, 'is_signed_var': False, 'len': 2, 'name':
  // 'BUS_Parking_Request_SW', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  int bus_parking_request_sw(const std::uint8_t* bytes,
                             const int32_t length) const;

  // config detail: {'bit': 4, 'is_signed_var': False, 'len': 2, 'name':
  // 'BUS_EngineStar_SW', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int bus_enginestar_sw(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 2, 'is_signed_var': False, 'len': 2, 'name':
  // 'BUS_Disable_DriveKeyboard', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  int bus_disable_drivekeyboard(const std::uint8_t* bytes,
                                const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 2, 'name':
  // 'BUS_Emergency_SW', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int bus_emergency_sw(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
