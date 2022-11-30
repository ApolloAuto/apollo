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

class Ecustatus4518 : public ::apollo::drivers::canbus::ProtocolData<
                          ::apollo::canbus::Ch> {
 public:
  static const int32_t ID;
  Ecustatus4518();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Ch* chassis) const override;

 private:
  // config detail: {'bit': 0, 'description': 'Ultrasonic detection distance 9
  // (Ultrasound status)', 'is_signed_var': False, 'len': 8, 'name':
  // 'ULTRASOUND_DIST_9', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|500]', 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
  double ultrasound_dist_9(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'bit': 8, 'description': 'Ultrasonic detection distance 10
  // (Ultrasound status)', 'is_signed_var': False, 'len': 8, 'name':
  // 'ULTRASOUND_DIST_10', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|500]', 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
  double ultrasound_dist_10(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'bit': 16, 'description': 'Ultrasonic detection distance 11
  // (Ultrasound status)', 'is_signed_var': False, 'len': 8, 'name':
  // 'ULTRASOUND_DIST_11', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|500]', 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
  double ultrasound_dist_11(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'bit': 24, 'description': 'Ultrasonic detection distance 12
  // (Ultrasound status)', 'is_signed_var': False, 'len': 8, 'name':
  // 'ULTRASOUND_DIST_12', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|500]', 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
  double ultrasound_dist_12(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'bit': 32, 'description': 'Ultrasonic detection distance 13
  // (Ultrasound status)', 'is_signed_var': False, 'len': 8, 'name':
  // 'ULTRASOUND_DIST_13', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|500]', 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
  double ultrasound_dist_13(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'bit': 40, 'description': 'Ultrasonic detection distance 14
  // (Ultrasound status)', 'is_signed_var': False, 'len': 8, 'name':
  // 'ULTRASOUND_DIST_14', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|500]', 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
  double ultrasound_dist_14(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'bit': 48, 'description': 'Ultrasonic detection distance 15
  // (Ultrasound status)', 'is_signed_var': False, 'len': 8, 'name':
  // 'ULTRASOUND_DIST_15', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|500]', 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
  double ultrasound_dist_15(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'bit': 56, 'description': 'Ultrasonic detection distance 16
  // (Ultrasound status)', 'is_signed_var': False, 'len': 8, 'name':
  // 'ULTRASOUND_DIST_16', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|500]', 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
  double ultrasound_dist_16(const std::uint8_t* bytes,
                            const int32_t length) const;
};

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
