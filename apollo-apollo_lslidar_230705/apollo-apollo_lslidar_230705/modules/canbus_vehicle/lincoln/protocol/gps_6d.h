/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

/**
 * @file gps_6d.h
 * @brief the class of Gps6d (for lincoln vehicle)
 */

#pragma once

#include "modules/canbus_vehicle/lincoln/proto/lincoln.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

/**
 * @namespace apollo::canbus::lincoln
 * @brief apollo::canbus::lincoln
 */
namespace apollo {
namespace canbus {
namespace lincoln {

/**
 * @class Gps6d
 *
 * @brief one of the protocol data of lincoln vehicle
 */
class Gps6d : public ::apollo::drivers::canbus::ProtocolData<
                  ::apollo::canbus::Lincoln> {
 public:
  static const int32_t ID;

  /*
   * @brief parse received data
   * @param bytes a pointer to the input bytes
   * @param length the length of the input bytes
   * @param timestamp the timestamp of input data
   * @param chassis_detail the parsed chassis_detail
   */
  virtual void Parse(const std::uint8_t *bytes, int32_t length,
                     Lincoln *chassis_detail) const;

  /**
   * @brief get latitude from byte array
   * config detail: {'name': 'latitude', 'offset': 0.0, 'precision':
   * 3.3333333e-07, 'len': 31, 'f_type': 'value', 'is_signed_var': True,
   * 'physical_range': '[0|0]', 'bit': 0, 'type': 'double', 'order': 'intel',
   * 'physical_unit': '"degrees"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of latitude
   */
  double latitude(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief get longitude from byte array
   * config detail: {'name': 'longitude', 'offset': 0.0, 'precision':
   * 3.3333333e-07, 'len': 31, 'f_type': 'value', 'is_signed_var': True,
   * 'physical_range': '[0|0]', 'bit': 32, 'type': 'double', 'order': 'intel',
   * 'physical_unit': '"degrees"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of longitude
   */
  double longitude(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief check is_valid from byte array
   * config detail: {'name': 'valid', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 63, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the boolean value of is_valid bit
   */
  bool is_valid(const std::uint8_t *bytes, int32_t length) const;
};

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
