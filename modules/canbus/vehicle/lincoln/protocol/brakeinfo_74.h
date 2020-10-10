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
 * @file brakeinfo_74.h
 * @brief the class of Brakeinfo74 (for lincoln vehicle)
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
 * @class Brakeinfo74
 *
 * @brief one of the protocol data of lincoln vehicle
 */
class Brakeinfo74 : public ::apollo::drivers::canbus::ProtocolData<
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
  void Parse(const std::uint8_t *bytes, int32_t length,
             ChassisDetail *chassis_detail) const override;

 private:
  /*
   * @brief get braking torque request
   * config detail: {'name': 'brktrqr', 'offset': 0.0, 'precision': 4.0, 'len':
   * 12, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
   * '[0|16380]', 'bit': 0, 'type': 'double', 'order': 'intel', 'physical_unit':
   * '"Nm"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of braking torque request
   */
  double braking_torque_request(const std::uint8_t *bytes,
                                int32_t length) const;
  /*
   * @brief get hill start assist status
   * config detail: {'name': 'hsastat', 'offset': 0.0, 'precision': 1.0, 'len':
   * 3, 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 12, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of the hill start assist status
   */
  int32_t hill_start_assist_status(const std::uint8_t *bytes,
                                   int32_t length) const;
  /*
   * @brief check vehicle stationary
   * config detail: {'name': 'statnry', 'offset': 0.0, 'precision': 1.0, 'len':
   * 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 15, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return boolean value of the vehicle stationary bit
   */
  bool is_vehicle_stationary(const std::uint8_t *bytes, int32_t length) const;

  /*
   * @brief get the actual braking torque
   * config detail: {'name': 'brktrqa', 'offset': 0.0, 'precision': 4.0, 'len':
   * 12, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
   * '[0|16380]', 'bit': 16, 'type': 'double', 'order': 'intel',
   * 'physical_unit': '"Nm"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of the actual braking torque
   */
  double braking_torque_actual(const std::uint8_t *bytes, int32_t length) const;

  /*
   * @brief get the hill start assist mode
   * config detail: {'name': 'hsamode', 'offset': 0.0, 'precision': 1.0, 'len':
   * 2, 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 28, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the mode of hill start assist
   */
  int32_t hill_start_assist_mode(const std::uint8_t *bytes,
                                 int32_t length) const;
  /*
   * @brief get the parking brake status
   * config detail: {'name': 'pbrake', 'offset': 0.0, 'precision': 1.0, 'len':
   * 2, 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 30, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of the parking brake status
   */
  int32_t parking_brake_status(const std::uint8_t *bytes, int32_t length) const;

  /*
   * @brief get the actual wheel torque
   * config detail: {'name': 'whltrq', 'offset': 0.0, 'precision': 4.0, 'len':
   * 14, 'f_type': 'value', 'is_signed_var': True, 'physical_range':
   * '[-32768|32764]', 'bit': 32, 'type': 'double', 'order': 'intel',
   * 'physical_unit': '"Nm"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of the actual wheel torque
   */
  double wheel_torque_actual(const std::uint8_t *bytes, int32_t length) const;

  /*
   * @brief get the acceleration over ground
   * config detail: {'name': 'aog', 'offset': 0.0, 'precision': 0.035, 'len':
   * 10, 'f_type': 'value', 'is_signed_var': True, 'physical_range':
   * '[-17.92|17.885]', 'bit': 48, 'type': 'double', 'order': 'intel',
   * 'physical_unit': '"m/s^2"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of the acceleration over ground
   */
  double acceleration_over_ground(const std::uint8_t *bytes,
                                  int32_t length) const;

  /*
   * @brief check abs active
   * config detail: {'name': 'absa', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 58, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return boolean value of the abs active bit
   */
  bool is_abs_active(const std::uint8_t *bytes, int32_t length) const;

  /*
   * @brief abs enabled
   * config detail: {'name': 'abse', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 59, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return boolean value of the abs enabled bit
   */
  bool is_abs_enabled(const std::uint8_t *bytes, int32_t length) const;

  /*
   * @brief check stability control active
   * config detail: {'name': 'staba', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 60, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return boolean value of the stability control active bit
   */
  bool is_stability_control_active(const std::uint8_t *bytes,
                                   int32_t length) const;
  /*
   * @brief check stability control enabled bit
   * config detail: {'name': 'stabe', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 61, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return boolean value of the stability control enabled bit
   */
  bool is_stability_control_enabled(const std::uint8_t *bytes,
                                    int32_t length) const;

  /*
   * @brief traction control active bit
   * config detail: {'name': 'traca', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 62, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return boolean value of the traction control active bit
   */
  bool is_traction_control_active(const std::uint8_t *bytes,
                                  int32_t length) const;
  /*
   * @brief traction control enabled bit
   * config detail: {'name': 'trace', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 63, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return boolean value of the traction control enabled bit
   */
  bool is_traction_control_enabled(const std::uint8_t *bytes,
                                   int32_t length) const;
};

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
