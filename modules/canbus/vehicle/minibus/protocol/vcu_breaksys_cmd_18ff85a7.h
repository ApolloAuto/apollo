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

class Vcubreaksyscmd18ff85a7 : public ::apollo::drivers::canbus::ProtocolData<
                                   ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Vcubreaksyscmd18ff85a7();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'bit': 23, 'is_signed_var': False, 'len': 1, 'name':
  // 'VCU_Brk_AutoParking_Request', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'bool'}
  bool vcu_brk_autoparking_request(const std::uint8_t* bytes,
                                   const int32_t length) const;

  // config detail: {'bit': 22, 'is_signed_var': False, 'len': 1, 'name':
  // 'VCU_Brk_Initivate_Enable', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'bool'}
  bool vcu_brk_initivate_enable(const std::uint8_t* bytes,
                                const int32_t length) const;

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name':
  // 'VCU_Brk_Right_Pressure', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|25.5]', 'physical_unit': 'MPa', 'precision': 0.1,
  // 'type': 'double'}
  double vcu_brk_right_pressure(const std::uint8_t* bytes,
                                const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name':
  // 'VCU_Brk_Left_Pressure', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|25.5]', 'physical_unit': 'MPa', 'precision': 0.1, 'type': 'double'}
  double vcu_brk_left_pressure(const std::uint8_t* bytes,
                               const int32_t length) const;
};

}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
