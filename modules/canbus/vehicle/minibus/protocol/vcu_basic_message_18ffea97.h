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

class Vcubasicmessage18ffea97 : public ::apollo::drivers::canbus::ProtocolData<
                                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Vcubasicmessage18ffea97();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  bool vcu_basic_onebit(const std::uint8_t* bytes, const int32_t length) const;

  bool vcu_basic_hp_halt(const std::uint8_t* bytes, const int32_t length) const;

  int vcu_basic_geatdefault_code(const std::uint8_t* bytes,
                                 const int32_t length) const;

  int vcu_basic_sysdefault_level(const std::uint8_t* bytes,
                                 const int32_t length) const;

  int vcu_basic_motocontroller_tempreture(const std::uint8_t* bytes,
                                          const int32_t length) const;

  int vcu_basic_motor_tempreture(const std::uint8_t* bytes,
                                 const int32_t length) const;

  int vcu_basic_charge_status(const std::uint8_t* bytes,
                              const int32_t length) const;

  int vcu_basic_real_highvoltage_sta(const std::uint8_t* bytes,
                                     const int32_t length) const;

  Vcu_basic_message_18ffea97::Vcu_basic_real_gearType vcu_basic_real_gear(
      const std::uint8_t* bytes, const int32_t length) const;

  double vcu_basic_motor_speed(const std::uint8_t* bytes,
                               const int32_t length) const;
};

}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
