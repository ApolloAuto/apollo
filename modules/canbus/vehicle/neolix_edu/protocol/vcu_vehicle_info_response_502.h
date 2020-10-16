/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
namespace neolix_edu {

class Vcuvehicleinforesponse502
    : public ::apollo::drivers::canbus::ProtocolData<
          ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Vcuvehicleinforesponse502();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'name': 'Vehicle_Softwareversion_Indicati', 'offset': 0.0,
  // 'precision': 1.0, 'len': 24, 'is_signed_var': False, 'physical_range':
  // '[0|16777215]', 'bit': 7, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': ''}
  int vehicle_softwareversion_indicati(const std::uint8_t* bytes,
                                       const int32_t length) const;

  // config detail: {'name': 'Project', 'offset': 0.0, 'precision': 1.0, 'len':
  // 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 27, 'type':
  // 'int', 'order': 'motorola', 'physical_unit': ''}
  int project(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Manufacturer', 'offset': 0.0, 'precision': 1.0,
  // 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 31,
  // 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int manufacturer(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Year', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 39, 'type':
  // 'int', 'order': 'motorola', 'physical_unit': ''}
  int year(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Month', 'offset': 0.0, 'precision': 1.0, 'len': 4,
  // 'is_signed_var': False, 'physical_range': '[1|12]', 'bit': 47, 'type':
  // 'int', 'order': 'motorola', 'physical_unit': ''}
  int month(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Day', 'offset': 0.0, 'precision': 1.0, 'len': 5,
  // 'is_signed_var': False, 'physical_range': '[1|31]', 'bit': 43, 'type':
  // 'int', 'order': 'motorola', 'physical_unit': ''}
  int day(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Vehicle_Serial_Number', 'offset': 0.0,
  // 'precision': 1.0, 'len': 15, 'is_signed_var': False, 'physical_range':
  // '[0|32767]', 'bit': 54, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': ''}
  int vehicle_serial_number(const std::uint8_t* bytes,
                            const int32_t length) const;
};

}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
