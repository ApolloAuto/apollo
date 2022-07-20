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
  int busv_emergencyhammer_worning(const std::uint8_t* bytes,
                                   const int32_t length) const;

  int busv_break_wear(const std::uint8_t* bytes, const int32_t length) const;

  int busv_body_notify(const std::uint8_t* bytes, const int32_t length) const;

  int busv_aircondition_status(const std::uint8_t* bytes,
                               const int32_t length) const;

  int busv_smalllamp_status(const std::uint8_t* bytes,
                            const int32_t length) const;

  Bus_vehicle_status_18fa0517::Busv_turnleft_stautsType busv_turnleft_stauts(
      const std::uint8_t* bytes, const int32_t length) const;

  Bus_vehicle_status_18fa0517::Busv_turnright_statusType busv_turnright_status(
      const std::uint8_t* bytes, const int32_t length) const;

  int busv_lowbeam_status(const std::uint8_t* bytes,
                          const int32_t length) const;

  Bus_vehicle_status_18fa0517::Busv_horn_statusType busv_horn_status(
      const std::uint8_t* bytes, const int32_t length) const;

  int busv_park_status(const std::uint8_t* bytes, const int32_t length) const;

  int busv_break_status(const std::uint8_t* bytes, const int32_t length) const;

  Bus_vehicle_status_18fa0517::Busv_fdoor_statusType busv_fdoor_status(
      const std::uint8_t* bytes, const int32_t length) const;

  int busv_reversing_status(const std::uint8_t* bytes,
                            const int32_t length) const;

  int bus_vehicle_sta_id(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
