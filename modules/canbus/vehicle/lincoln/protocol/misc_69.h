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
 * @file misc_69.h
 * @brief the class of Misc69 (for lincoln vehicle)
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
 * @class Misc69
 *
 * @brief one of the protocol data of lincoln vehicle
 */
class Misc69 : public ::apollo::drivers::canbus::ProtocolData<
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
   * @brief calculate the turn signal status based on byte array.
   * config detail: {'name': 'trnstat', 'offset': 0.0, 'precision': 1.0, 'len':
   * 2, 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of turn signal status
   */
  int32_t turn_signal_status(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief calculate the high beam status based on byte array.
   * config detail: {'name': 'hibeam', 'offset': 0.0, 'precision': 1.0, 'len':
   * 2, 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 2, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of high beam status
   */
  int32_t high_beam_status(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief calculate the wiper status based on byte array.
   * config detail: {'name': 'wiper', 'offset': 0.0, 'precision': 1.0, 'len': 4,
   * 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 4, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of wiper status
   */
  int32_t wiper_status(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief calculate the ambient light status based on byte array.
   * config detail: {'name': 'ambient', 'offset': 0.0, 'precision': 1.0, 'len':
   * 3, 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 8, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of ambient light
   */
  int32_t ambient_light_status(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief check acc on pressed bit based on byte array.
   * config detail: {'name': 'on', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 11, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the boolean value of acc on pressed bit
   */
  bool is_acc_on_pressed(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief check acc off pressed bit based on byte array.
   * config detail: {'name': 'off', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 12, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of acc off pressed bit
   */
  bool is_acc_off_pressed(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief check acc resume pressed bit based on byte array.
   * config detail: {'name': 'res', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 13, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the boolean value of acc resume pressed bit
   */
  bool is_acc_resume_pressed(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief check acc cancel pressed bit based on byte array.
   * config detail: {'name': 'cncl', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 14, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the boolean value of acc cancel pressed bit
   */
  bool is_acc_cancel_pressed(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief check acc on or off pressed bit based on byte array.
   * config detail: {'name': 'onoff', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 16, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the boolean value of acc on or off pressed bit
   */
  bool is_acc_on_or_off_pressed(const std::uint8_t *bytes,
                                int32_t length) const;

  /**
   * @brief check acc resume or cancel pressed bit based on byte array.
   * config detail: {'name': 'rescncl', 'offset': 0.0, 'precision': 1.0, 'len':
   * 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 17, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the boolean value of acc resume or cancel pressed bit
   */
  bool is_acc_resume_or_cancel_pressed(const std::uint8_t *bytes,
                                       int32_t length) const;

  /**
   * @brief check the acc increment set speed pressed bit based on byte array.
   * config detail: {'name': 'sinc', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 18, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the boolean value of acc increment set speed pressed bit
   */
  bool is_acc_increment_set_speed_pressed(const std::uint8_t *bytes,
                                          int32_t length) const;

  /**
   * @brief check the acc decrement set speed pressed bit based on byte array.
   * config detail: {'name': 'sdec', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 19, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the boolean value of acc decrement set speed pressed bit
   */
  bool is_acc_decrement_set_speed_pressed(const std::uint8_t *bytes,
                                          int32_t length) const;

  /**
   * @brief check the acc increment following gap pressed bit based on byte
   * array.
   * config detail: {'name': 'ginc', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 20, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the boolean value of acc increment following gap pressed bit
   */
  bool is_acc_increment_following_gap_pressed(const std::uint8_t *bytes,
                                              int32_t length) const;

  /**
   * @brief check the acc decrement following gap pressed bit based on byte
   * array.
   * config detail: {'name': 'gdec', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 21, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the boolean value of acc decrement following gap pressed bit
   */
  bool is_acc_decrement_following_gap_pressed(const std::uint8_t *bytes,
                                              int32_t length) const;

  /**
   * @brief check the lka on or off pressed bit based on byte array.
   * config detail: {'name': 'lkaen', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 22, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the boolean value of lka on or off pressed bit
   */
  bool is_lka_on_or_off_pressed(const std::uint8_t *bytes,
                                int32_t length) const;

  /**
   * @brief check the canbus fault bit based on byte array.
   * config detail: {'name': 'fltbus', 'offset': 0.0, 'precision': 1.0, 'len':
   * 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 23, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the boolean value of canbus fault bit
   */
  bool is_canbus_fault(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief check the driver door open bit based on byte array.
   * config detail: {'name': 'doord', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 24, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the boolean value of driver door open bit
   */
  bool is_driver_door_open(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief check the passenger door open bit based on byte array.
   * config detail: {'name': 'doorp', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 25, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the boolean value of passenger door open bit
   */
  bool is_passenger_door_open(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief check the passenger door open bit based on byte array.
   * config detail: {'name': 'doorl', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 26, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the boolean value of rear left door open bit
   */
  bool is_rear_left_door_open(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief check the rear right door open bit based on byte array.
   * config detail: {'name': 'doorr', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 27, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the boolean value of rear right door open bit
   */
  bool is_rear_right_door_open(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief check the hood open bit based on byte array.
   * config detail: {'name': 'hood', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 28, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the boolean value of hood open bit
   */
  bool is_hood_open(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief check the trunk open bit based on byte array.
   * config detail: {'name': 'trunk', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 29, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the boolean value of trunk open bit
   */
  bool is_trunk_open(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief check the passenger detected bit based on byte array.
   * config detail: {'name': 'pdect', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 30, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the boolean value of passenger detected bit
   */
  bool is_passenger_detected(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief check the passenger airbag enabled bit based on byte array.
   * config detail: {'name': 'pabag', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 31, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the boolean value of passenger airbag enabled bit
   */
  bool is_passenger_airbag_enabled(const std::uint8_t *bytes,
                                   int32_t length) const;

  /**
   * @brief check the driver belt buckled bit based on byte array.
   * config detail: {'name': 'beltd', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 32, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the boolean value of driver belt buckled bit
   */
  bool is_driver_belt_buckled(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief check the passenger belt buckled bit based on byte array.
   * config detail: {'name': 'beltp', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 33, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the boolean value of passenger belt buckled bit
   */
  bool is_passenger_belt_buckled(const std::uint8_t *bytes,
                                 int32_t length) const;
};

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
