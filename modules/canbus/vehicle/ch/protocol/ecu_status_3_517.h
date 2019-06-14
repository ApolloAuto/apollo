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
namespace ch {

class Ecustatus3517 : public ::apollo::drivers::canbus::ProtocolData<
                          ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Ecustatus3517();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'description': 'Ultrasonic detection distance 1 (Ultrasound
  // status)', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name':
  // 'ULTRASOUND_DIST_1', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': 'cm'}
  int ultrasound_dist_1(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Ultrasonic detection distance 2 (Ultrasound
  // status)', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name':
  // 'ULTRASOUND_DIST_2', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 8, 'type': 'int', 'order': 'intel', 'physical_unit': 'cm'}
  int ultrasound_dist_2(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Ultrasonic detection distance 3 (Ultrasound
  // status)', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name':
  // 'ULTRASOUND_DIST_3', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 16, 'type': 'int', 'order': 'intel', 'physical_unit': 'cm'}
  int ultrasound_dist_3(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Ultrasonic detection distance 4 (Ultrasound
  // status)', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name':
  // 'ULTRASOUND_DIST_4', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 24, 'type': 'int', 'order': 'intel', 'physical_unit': 'cm'}
  int ultrasound_dist_4(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Ultrasonic detection distance 5 (Ultrasound
  // status)', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name':
  // 'ULTRASOUND_DIST_5', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 32, 'type': 'int', 'order': 'intel', 'physical_unit': 'cm'}
  int ultrasound_dist_5(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Ultrasonic detection distance 6 (Ultrasound
  // status)', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name':
  // 'ULTRASOUND_DIST_6', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 40, 'type': 'int', 'order': 'intel', 'physical_unit': 'cm'}
  int ultrasound_dist_6(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Ultrasonic detection distance 7 (Ultrasound
  // status)', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name':
  // 'ULTRASOUND_DIST_7', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 48, 'type': 'int', 'order': 'intel', 'physical_unit': 'cm'}
  int ultrasound_dist_7(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Ultrasonic detection distance 8 (Ultrasound
  // status)', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name':
  // 'ULTRASOUND_DIST_8', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': 'cm'}
  int ultrasound_dist_8(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
