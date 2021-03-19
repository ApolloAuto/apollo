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
 * @file gps_6e.h
 * @brief the class of Gps6e (for lincoln vehicle)
 */

#pragma once

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

/**
 * @namespace apollo::canbus::lincoln
 * @brief apollo::canbus::lincoln
 */
namespace apollo {
namespace canbus {
namespace lincoln {

/**
 * @class Gps6e
 *
 * @brief one of the protocol data of lincoln vehicle
 */
class Gps6e : public ::apollo::drivers::canbus::ProtocolData<
                  ::apollo::canbus::ChassisDetail> {
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
                     ChassisDetail *chassis_detail) const;

  /**
   * @brief get year from byte array
   * config detail: {'name': 'year', 'offset': 0.0, 'precision': 1.0, 'len': 7,
   * 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|99]',
   * 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': '"years"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of year
   */
  int32_t year(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief get month from byte array
   * config detail: {'name': 'month', 'offset': 0.0, 'precision': 1.0, 'len': 4,
   * 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[1|12]',
   * 'bit': 8, 'type': 'int', 'order': 'intel', 'physical_unit': '"months"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of month
   */
  int32_t month(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief get day from byte array
   * config detail: {'name': 'day', 'offset': 0.0, 'precision': 1.0, 'len': 5,
   * 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[1|31]',
   * 'bit': 16, 'type': 'int', 'order': 'intel', 'physical_unit': '"days"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of day
   */
  int32_t day(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief get hours from byte array
   * config detail: {'name': 'hours', 'offset': 0.0, 'precision': 1.0, 'len': 5,
   * 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|23]',
   * 'bit': 24, 'type': 'int', 'order': 'intel', 'physical_unit': '"hours"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of hours
   */
  int32_t hours(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief get minutes from byte array
   * config detail: {'name': 'minutes', 'offset': 0.0, 'precision': 1.0, 'len':
   * 6, 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|59]',
   * 'bit': 32, 'type': 'int', 'order': 'intel', 'physical_unit': '"minutes"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of minutes
   */
  int32_t minutes(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief get year from byte array
   * config detail: {'name': 'seconds', 'offset': 0.0, 'precision': 1.0, 'len':
   * 6, 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|59]',
   * 'bit': 40, 'type': 'int', 'order': 'intel', 'physical_unit': '"seconds"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of minutes
   */
  int32_t seconds(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief get compass direction from byte array
   * config detail: {'name': 'compass', 'offset': 0.0, 'precision': 45.0, 'len':
   * 4, 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|315]',
   * 'bit': 48, 'type': 'double', 'order': 'intel', 'physical_unit':
   * '"degrees"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of compass direction
   */
  double compass_direction(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief get pdop from byte array
   * config detail: {'name': 'pdop', 'offset': 0.0, 'precision': 0.2, 'len': 5,
   * 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 56, 'type': 'double', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of pdop
   */
  double pdop(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief check gps fault from byte array
   * config detail: {'name': 'fltgps', 'offset': 0.0, 'precision': 1.0, 'len':
   * 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 61, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of gps fault
   */
  bool is_gps_fault(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief get inferred position from byte array
   * config detail: {'name': 'inf', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 62, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of inferred position
   */
  bool is_inferred_position(const std::uint8_t *bytes, int32_t length) const;
};

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
