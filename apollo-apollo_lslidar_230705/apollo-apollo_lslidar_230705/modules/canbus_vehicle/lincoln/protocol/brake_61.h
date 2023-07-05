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
 * @file brake_61.h
 * @brief the class of Brake61 (for lincoln vehicle)
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
 * @class Brake61
 *
 * @brief one of the protocol data of lincoln vehicle
 */
class Brake61 : public ::apollo::drivers::canbus::ProtocolData<
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

 private:
  /**
   * @brief get pedal input from byte array
   * config detail: {'name': 'pi', 'offset': 0.0, 'precision':
   * 1.52590218966964e-05, 'len': 16, 'f_type': 'value', 'is_signed_var': False,
   * 'physical_range': '[0|1]', 'bit': 0, 'type': 'double', 'order': 'intel',
   * 'physical_unit': '"%"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of pedal input
   */
  double pedal_input(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief get pedal command for control from byte array
   * config detail: {'name': 'pc', 'offset': 0.0, 'precision':
   * 1.52590218966964e-05, 'len': 16, 'f_type': 'value', 'is_signed_var': False,
   * 'physical_range': '[0|1]', 'bit': 16, 'type': 'double', 'order': 'intel',
   * 'physical_unit': '"%"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of pedal command for control
   */
  double pedal_cmd(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief get pedal output from byte array
   * config detail: {'name': 'po', 'offset': 0.0, 'precision':
   * 1.52590218966964e-05, 'len': 16, 'f_type': 'value', 'is_signed_var': False,
   * 'physical_range': '[0|1]', 'bit': 32, 'type': 'double', 'order': 'intel',
   * 'physical_unit': '"%"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of pedal command for control
   */
  double pedal_output(const std::uint8_t *bytes, int32_t length) const;

  double parse_two_frames(const std::uint8_t low_byte,
                          const std::uint8_t high_byte) const;

  /**
   * @brief check if boo bit from input byte array is 1 or 0 (at position 0)
   * config detail: {'name': 'bi', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 50, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return boolean value of the corresponding bit
   */
  bool boo_input(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief check if cmd bit from input byte array is 1 or 0 (at position 1)
   * config detail: {'name': 'bc', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 49, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return boolean value of the corresponding bit
   */
  bool boo_cmd(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief check the boo bit for output from byte array (at position 2)
   * config detail: {'name': 'bo', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 48, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return boolean value of the corresponding bit
   */
  bool boo_output(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief check watchdog_counter bit (at position 3)
   * config detail: {'name': 'wdcbrk', 'offset': 0.0, 'precision': 1.0, 'len':
   * 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 51, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return boolean value of the corresponding bit
   */
  bool is_watchdog_counter_applying_brakes(const std::uint8_t *bytes,
                                           int32_t length) const;

  /**
   * @brief check watchdog_counter bit (at position 4)
   * config detail: {'name': 'wdcsrc', 'offset': 0.0, 'precision': 1.0, 'len':
   * 4, 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 52, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return boolean value of the corresponding bit
   */
  int32_t watchdog_counter_source(const std::uint8_t *bytes,
                                  int32_t length) const;

  /**
   * @brief check if enabled
   * config detail: {'name': 'en', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 56, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return true when enabled
   */
  bool is_enabled(const std::uint8_t *bytes, int32_t length) const;

  /**
   * brief check driver override
   * config detail: {'name': 'override', 'offset': 0.0, 'precision': 1.0, 'len':
   * 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 57, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return true if driver override
   */
  bool is_driver_override(const std::uint8_t *bytes, int32_t length) const;

  /**
   * brief check if is_driver activity
   * config detail: {'name': 'driver', 'offset': 0.0, 'precision': 1.0, 'len':
   * 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 58, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return true if driving activity
   */
  bool is_driver_activity(const std::uint8_t *bytes, int32_t length) const;

  /**
   * brief check if is watchdog counter fault
   * config detail: {'name': 'fltwdc', 'offset': 0.0, 'precision': 1.0, 'len':
   * 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 59, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return true if watchdog counter fault
   */
  bool is_watchdog_counter_fault(const std::uint8_t *bytes,
                                 int32_t length) const;

  /**
   * brief check if is channel 1 fault
   * config detail: {'name': 'flt1', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 60, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return true if channel 1 fault
   */
  bool is_channel_1_fault(const std::uint8_t *bytes, int32_t length) const;

  /**
   * brief check if is channel 2 fault
   * config detail: {'name': 'flt2', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 61, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return true if channel 2 fault
   */
  bool is_channel_2_fault(const std::uint8_t *bytes, int32_t length) const;

  /**
   * brief check if boo switch fault
   * config detail: {'name': 'fltboo', 'offset': 0.0, 'precision': 1.0, 'len':
   * 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 62, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return true if boo switch fault
   */
  bool is_boo_switch_fault(const std::uint8_t *bytes, int32_t length) const;

  /**
   * brief check if connector fault
   * config detail: {'name': 'fltcon', 'offset': 0.0, 'precision': 1.0, 'len':
   * 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 63, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return true if connector fault
   */
  bool is_connector_fault(const std::uint8_t *bytes, int32_t length) const;
};

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
