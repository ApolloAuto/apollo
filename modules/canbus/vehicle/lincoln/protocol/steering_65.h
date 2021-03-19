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
 * @file steering_65.h
 * @brief the class of steering_65.h (for lincoln vehicle)
 */

#pragma once

#include <sys/time.h>

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
 * @class Steering65
 *
 * @brief one of the protocol data of lincoln vehicle
 */
class Steering65 : public ::apollo::drivers::canbus::ProtocolData<
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

  /*
   * @brief parse received data
   * @param bytes a pointer to the input bytes
   * @param length the length of the input bytes
   * @param timestamp the timestamp of input data
   * @param chassis_detail the parsed chassis_detail
   */
  virtual void Parse(const std::uint8_t *bytes, int32_t length,
                     const struct timeval &timestamp,
                     ChassisDetail *chassis_detail) const;

  /**
   * @brief calculate steering angle based on byte array.
   * config detail: {'name': 'angle', 'offset': 0.0, 'precision': 0.1, 'len':
   * 16, 'f_type': 'value', 'is_signed_var': True, 'physical_range':
   * '[-470|470]', 'bit': 0, 'type': 'double', 'order': 'intel',
   * 'physical_unit': '"degrees"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of steering angle
   */
  double steering_angle(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief calculate reported steering angle command based on byte array.
   * config detail: {'name': 'cmd', 'offset': 0.0, 'precision': 0.1, 'len': 16,
   * 'f_type': 'value', 'is_signed_var': True, 'physical_range': '[-470|470]',
   * 'bit': 16, 'type': 'double', 'order': 'intel', 'physical_unit':
   * '"degrees"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of reported steering angle command
   */
  double reported_steering_angle_cmd(const std::uint8_t *bytes,
                                     int32_t length) const;

  /**
   * @brief calculate vehicle speed based on byte array.
   * config detail: {'name': 'speed', 'offset': 0.0, 'precision': 0.01, 'len':
   * 16, 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 32, 'type': 'double', 'order': 'intel', 'physical_unit': '"kph"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of vehicle speed
   */
  double vehicle_speed(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief calculate epas torque based on byte array.
   * config detail: {'name': 'torque', 'offset': 0.0, 'precision': 0.0625,
   * 'len': 8, 'f_type': 'value', 'is_signed_var': True, 'physical_range':
   * '[0|0]', 'bit': 48, 'type': 'double', 'order': 'intel', 'physical_unit':
   * '"Nm"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of epas torque
   */
  double epas_torque(const std::uint8_t *bytes, int32_t length) const;

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
   * @brief check watchdog counter fault based on byte array.
   * config detail: {'name': 'fltwdc', 'offset': 0.0, 'precision': 1.0, 'len':
   * 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 59, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the boolean value of watchdog counter fault
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
   * @return the boolean value of steering angle
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
   * @brief check calibration fault bit based on byte array.
   * config detail: {'name': 'fltcal', 'offset': 0.0, 'precision': 1.0, 'len':
   * 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 62, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the boolean value of calibration fault bit
   */
  bool is_calibration_fault(const std::uint8_t *bytes, int32_t length) const;

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
