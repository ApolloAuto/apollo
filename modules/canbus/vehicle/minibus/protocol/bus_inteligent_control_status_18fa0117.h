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
  int bus_innerdoor_daysensor(const std::uint8_t* bytes,
                              const int32_t length) const;

  int bus_outdoor_daysensor(const std::uint8_t* bytes,
                            const int32_t length) const;

  int bus_outdoor_key(const std::uint8_t* bytes, const int32_t length) const;

  int bus_innerdoor_key(const std::uint8_t* bytes, const int32_t length) const;

  int bus_breaking_key(const std::uint8_t* bytes, const int32_t length) const;

  int bus_reversing_key(const std::uint8_t* bytes, const int32_t length) const;

  int bus_drive_key(const std::uint8_t* bytes, const int32_t length) const;

  int bus_joystick_right_key(const std::uint8_t* bytes,
                             const int32_t length) const;

  int bus_joystick_left_key(const std::uint8_t* bytes,
                            const int32_t length) const;

  int bus_joystick_behind_key(const std::uint8_t* bytes,
                              const int32_t length) const;

  int bus_joystick_front_key(const std::uint8_t* bytes,
                             const int32_t length) const;

  int bus_parking_request_sw(const std::uint8_t* bytes,
                             const int32_t length) const;

  int bus_enginestar_sw(const std::uint8_t* bytes, const int32_t length) const;

  int bus_disable_drivekeyboard(const std::uint8_t* bytes,
                                const int32_t length) const;

  int bus_emergency_sw(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
