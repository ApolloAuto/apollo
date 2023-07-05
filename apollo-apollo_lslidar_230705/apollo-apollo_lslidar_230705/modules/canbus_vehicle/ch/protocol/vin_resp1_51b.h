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

#include <string>

#include "modules/canbus_vehicle/ch/proto/ch.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace ch {

class Vinresp151b : public ::apollo::drivers::canbus::ProtocolData<
                        ::apollo::canbus::Ch> {
 public:
  static const int32_t ID;
  Vinresp151b();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Ch* chassis) const override;

 private:
  // config detail: {'bit': 56, 'description': 'VIN Response', 'is_signed_var':
  // False, 'len': 8, 'name': 'VIN08', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  std::string vin08(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 48, 'description': 'VIN Response', 'is_signed_var':
  // False, 'len': 8, 'name': 'VIN07', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  std::string vin07(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 40, 'description': 'VIN Response', 'is_signed_var':
  // False, 'len': 8, 'name': 'VIN06', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  std::string vin06(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 32, 'description': 'VIN Response', 'is_signed_var':
  // False, 'len': 8, 'name': 'VIN05', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  std::string vin05(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 24, 'description': 'VIN Response', 'is_signed_var':
  // False, 'len': 8, 'name': 'VIN04', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  std::string vin04(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 16, 'description': 'VIN Response', 'is_signed_var':
  // False, 'len': 8, 'name': 'VIN03', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  std::string vin03(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 8, 'description': 'VIN Response', 'is_signed_var':
  // False, 'len': 8, 'name': 'VIN02', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  std::string vin02(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'description': 'VIN Response', 'is_signed_var':
  // False, 'len': 8, 'name': 'VIN01', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  std::string vin01(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
