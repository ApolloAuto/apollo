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

#include "modules/canbus_vehicle/ge3/proto/ge3.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace ge3 {

class Scu2302 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::Ge3> {
 public:
  static const int32_t ID;
  Scu2302();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Ge3* chassis) const override;

 private:
  // config detail: {'description': 'VIN string character 07', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'name': 'VIN07', 'is_signed_var': False,
  // 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': '-'}
  int vin07(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'VIN string character 06', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'name': 'VIN06', 'is_signed_var': False,
  // 'physical_range': '[0|255]', 'bit': 55, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': '-'}
  int vin06(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'VIN string character 05', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'name': 'VIN05', 'is_signed_var': False,
  // 'physical_range': '[0|255]', 'bit': 47, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': '-'}
  int vin05(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'VIN string character 04', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'name': 'VIN04', 'is_signed_var': False,
  // 'physical_range': '[0|255]', 'bit': 39, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': '-'}
  int vin04(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'VIN string character 03', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'name': 'VIN03', 'is_signed_var': False,
  // 'physical_range': '[0|255]', 'bit': 31, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': '-'}
  int vin03(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'VIN string character 02', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'name': 'VIN02', 'is_signed_var': False,
  // 'physical_range': '[0|255]', 'bit': 23, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': '-'}
  int vin02(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'VIN string character 01', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'name': 'VIN01', 'is_signed_var': False,
  // 'physical_range': '[0|255]', 'bit': 15, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': '-'}
  int vin01(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'VIN string character 00', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'name': 'VIN00', 'is_signed_var': False,
  // 'physical_range': '[0|255]', 'bit': 7, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': '-'}
  int vin00(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
