/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_CANBUS_VEHICLE_GEM_PROTOCOL_LAT_LON_HEADING_RPT_82_H_
#define MODULES_CANBUS_VEHICLE_GEM_PROTOCOL_LAT_LON_HEADING_RPT_82_H_

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace gem {

class Latlonheadingrpt82 : public ::apollo::drivers::canbus::ProtocolData<
                               ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Latlonheadingrpt82();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'name': 'HEADING', 'offset': 0.0, 'precision': 0.01, 'len':
  // 16, 'is_signed_var': True, 'physical_range': '[-327.68|327.67]', 'bit': 55,
  // 'type': 'double', 'order': 'motorola', 'physical_unit': 'deg'}
  double heading(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'LONGITUDE_SECONDS', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': True, 'physical_range':
  // '[-128|127]', 'bit': 47, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': 'sec'}
  int longitude_seconds(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'LONGITUDE_MINUTES', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': True, 'physical_range':
  // '[-128|127]', 'bit': 39, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': 'min'}
  int longitude_minutes(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'LONGITUDE_DEGREES', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': True, 'physical_range':
  // '[-128|127]', 'bit': 31, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': 'deg'}
  int longitude_degrees(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'LATITUDE_SECONDS', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': True, 'physical_range':
  // '[-128|127]', 'bit': 23, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': 'sec'}
  int latitude_seconds(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'LATITUDE_MINUTES', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': True, 'physical_range':
  // '[-128|127]', 'bit': 15, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': 'min'}
  int latitude_minutes(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'LATITUDE_DEGREES', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': True, 'physical_range':
  // '[-128|127]', 'bit': 7, 'type': 'int', 'order': 'motorola',
  // 'physical_unit': 'deg'}
  int latitude_degrees(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace gem
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_GEM_PROTOCOL_LAT_LON_HEADING_RPT_82_H_
