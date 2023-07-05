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
 * @file gps_6f.h
 * @brief the class of Gps6f (for lincoln vehicle)
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
 * @class Gps6f
 *
 * @brief one of the protocol data of lincoln vehicle
 */
class Gps6f : public ::apollo::drivers::canbus::ProtocolData<
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
   * @brief get altitude from byte array
   * config detail: {'name': 'altitude', 'offset': 0.0, 'precision': 0.25,
   * 'len': 16, 'f_type': 'value', 'is_signed_var': True, 'physical_range':
   * '[0|0]', 'bit': 0, 'type': 'double', 'order': 'intel', 'physical_unit':
   * '"m"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of altitude
   */
  double altitude(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief get heading from byte array
   * config detail: {'name': 'heading', 'offset': 0.0, 'precision': 0.01, 'len':
   * 16, 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 16, 'type': 'double', 'order': 'intel', 'physical_unit':
   * '"degrees"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of heading
   */
  double heading(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief get speed from byte array
   * config detail: {'name': 'speed', 'offset': 0.0, 'precision': 1.0, 'len': 8,
   * 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 32, 'type': 'int', 'order': 'intel', 'physical_unit': '"mph"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of speed
   */
  int32_t speed(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief get hdop from byte array
   * config detail: {'name': 'hdop', 'offset': 0.0, 'precision': 0.2, 'len': 5,
   * 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 40, 'type': 'double', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of hdop
   */
  double hdop(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief get vdop from byte array
   * config detail: {'name': 'vdop', 'offset': 0.0, 'precision': 0.2, 'len': 5,
   * 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 48, 'type': 'double', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of vdop
   */
  double vdop(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief get fix quality from byte array
   * config detail: {'name': 'quality', 'offset': 0.0, 'precision': 1.0, 'len':
   * 3, 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of fix quality
   */
  int32_t fix_quality(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief get number of satellites from byte array
   * config detail: {'name': 'numsat', 'offset': 0.0, 'precision': 1.0, 'len':
   * 5, 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 59, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the number of satellites
   */
  int32_t num_satellites(const std::uint8_t *bytes, int32_t length) const;
};

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
