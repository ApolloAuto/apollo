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
 * @file throttle_63.h
 * @brief the class of Throttle63 (for lincoln vehicle)
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
 * @class Throttle63
 *
 * @brief one of the protocol data of lincoln vehicle
 */
class Throttle63 : public ::apollo::drivers::canbus::ProtocolData<
                       ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  /*
   * @brief parse received data
   * @param bytes a pointer to the input bytes
   * @param length the length of the input bytes
   * @param chassis_detail the parsed chassis_detail
   */
  virtual void Parse(const std::uint8_t *bytes, int32_t length,
                     ChassisDetail *chassis_detail) const;

  /**
   * @brief calculate pedal input based on byte array.
   * config detail: {'name': 'pi', 'offset': 0.0, 'precision':
   * 1.52590218966964e-05, 'len': 16, 'f_type': 'value', 'is_signed_var': False,
   * 'physical_range': '[0|1]', 'bit': 0, 'type': 'double', 'order': 'intel',
   * 'physical_unit': '"%"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of byte input
   */
  double pedal_input(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief calculate pedal command based on byte array.
   * config detail: {'name': 'pc', 'offset': 0.0, 'precision':
   * 1.52590218966964e-05, 'len': 16, 'f_type': 'value', 'is_signed_var': False,
   * 'physical_range': '[0|1]', 'bit': 16, 'type': 'double', 'order': 'intel',
   * 'physical_unit': '"%"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of pedal command
   */
  double pedal_cmd(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief calculate pedal output based on byte array.
   * config detail: {'name': 'po', 'offset': 0.0, 'precision':
   * 1.52590218966964e-05, 'len': 16, 'f_type': 'value', 'is_signed_var': False,
   * 'physical_range': '[0|1]', 'bit': 32, 'type': 'double', 'order': 'intel',
   * 'physical_unit': '"%"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of pedal output
   */
  double pedal_output(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief calculate watchdog counter source based on byte array.
   * config detail: {'name': 'wdcsrc', 'offset': 0.0, 'precision': 1.0, 'len':
   * 4, 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 52, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of watchdog counter source
   */
  int32_t watchdog_counter_source(const std::uint8_t *bytes,
                                  int32_t length) const;

  /**
   * @brief check enabled bit based on byte array.
   * config detail: {'name': 'en', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 56, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the boolean value of enabled bit
   */
  bool is_enabled(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief check driver override bit based on byte array.
   * config detail: {'name': 'override', 'offset': 0.0, 'precision': 1.0, 'len':
   * 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 57, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the boolean value of driver override
   */
  bool is_driver_override(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief check driver activity bit based on byte array.
   * config detail: {'name': 'driver', 'offset': 0.0, 'precision': 1.0, 'len':
   * 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 58, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the boolean value of driver activity
   */
  bool is_driver_activity(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief check watchdog counter fault bit based on byte array.
   * config detail: {'name': 'fltwdc', 'offset': 0.0, 'precision': 1.0, 'len':
   * 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 59, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the boolean value of watchdog counter fault bit
   */
  bool is_watchdog_counter_fault(const std::uint8_t *bytes,
                                 int32_t length) const;

  /**
   * @brief check channel 1 fault bit based on byte array.
   * config detail: {'name': 'flt1', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 60, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the boolean value of channel 1 fault bit
   */
  bool is_channel_1_fault(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief check channel 2 fault bit based on byte array.
   * config detail: {'name': 'flt2', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 61, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the boolean value of channel 2 fault bit
   */
  bool is_channel_2_fault(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief check connector fault bit based on byte array.
   * config detail: {'name': 'fltcon', 'offset': 0.0, 'precision': 1.0, 'len':
   * 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 63, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the boolean value of connector fault bit
   */
  bool is_connector_fault(const std::uint8_t *bytes, int32_t length) const;
};

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
