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

class Vinresp251c : public ::apollo::drivers::canbus::ProtocolData<
                        ::apollo::canbus::Ch> {
 public:
  static const int32_t ID;
  Vinresp251c();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Ch* chassis) const override;

 private:
  // config detail: {'bit': 56, 'description': 'VIN Response', 'is_signed_var':
  // False, 'len': 8, 'name': 'VIN16', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  std::string vin16(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 48, 'description': 'VIN Response', 'is_signed_var':
  // False, 'len': 8, 'name': 'VIN15', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  std::string vin15(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 40, 'description': 'VIN Response', 'is_signed_var':
  // False, 'len': 8, 'name': 'VIN14', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  std::string vin14(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 32, 'description': 'VIN Response', 'is_signed_var':
  // False, 'len': 8, 'name': 'VIN13', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  std::string vin13(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 24, 'description': 'VIN Response', 'is_signed_var':
  // False, 'len': 8, 'name': 'VIN12', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  std::string vin12(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 16, 'description': 'VIN Response', 'is_signed_var':
  // False, 'len': 8, 'name': 'VIN11', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  std::string vin11(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 8, 'description': 'VIN Response', 'is_signed_var':
  // False, 'len': 8, 'name': 'VIN10', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  std::string vin10(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'description': 'VIN Response', 'is_signed_var':
  // False, 'len': 8, 'name': 'VIN09', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  std::string vin09(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
