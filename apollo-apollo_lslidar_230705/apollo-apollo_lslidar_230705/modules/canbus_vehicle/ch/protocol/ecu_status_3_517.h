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

#include "modules/canbus_vehicle/ch/proto/ch.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace ch {

class Ecustatus3517 : public ::apollo::drivers::canbus::ProtocolData<
                          ::apollo::canbus::Ch> {
 public:
  static const int32_t ID;
  Ecustatus3517();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Ch* chassis) const override;

 private:
  // config detail: {'bit': 0, 'description': 'Ultrasonic detection distance 1
  // (Ultrasound status)', 'is_signed_var': False, 'len': 8, 'name':
  // 'ULTRASOUND_DIST_1', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|500]', 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
  double ultrasound_dist_1(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'bit': 8, 'description': 'Ultrasonic detection distance 2
  // (Ultrasound status)', 'is_signed_var': False, 'len': 8, 'name':
  // 'ULTRASOUND_DIST_2', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|500]', 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
  double ultrasound_dist_2(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'bit': 16, 'description': 'Ultrasonic detection distance 3
  // (Ultrasound status)', 'is_signed_var': False, 'len': 8, 'name':
  // 'ULTRASOUND_DIST_3', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|500]', 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
  double ultrasound_dist_3(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'bit': 24, 'description': 'Ultrasonic detection distance 4
  // (Ultrasound status)', 'is_signed_var': False, 'len': 8, 'name':
  // 'ULTRASOUND_DIST_4', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|500]', 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
  double ultrasound_dist_4(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'bit': 32, 'description': 'Ultrasonic detection distance 5
  // (Ultrasound status)', 'is_signed_var': False, 'len': 8, 'name':
  // 'ULTRASOUND_DIST_5', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|500]', 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
  double ultrasound_dist_5(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'bit': 40, 'description': 'Ultrasonic detection distance 6
  // (Ultrasound status)', 'is_signed_var': False, 'len': 8, 'name':
  // 'ULTRASOUND_DIST_6', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|500]', 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
  double ultrasound_dist_6(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'bit': 48, 'description': 'Ultrasonic detection distance 7
  // (Ultrasound status)', 'is_signed_var': False, 'len': 8, 'name':
  // 'ULTRASOUND_DIST_7', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|500]', 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
  double ultrasound_dist_7(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'bit': 56, 'description': 'Ultrasonic detection distance 8
  // (Ultrasound status)', 'is_signed_var': False, 'len': 8, 'name':
  // 'ULTRASOUND_DIST_8', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|500]', 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
  double ultrasound_dist_8(const std::uint8_t* bytes,
                           const int32_t length) const;
};

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
