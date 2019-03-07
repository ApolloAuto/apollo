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
 * @file gear_67.h
 * @brief the class of Gear67 (for lincoln vehicle)
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
 * @class Gear67
 *
 * @brief one of the protocol data of lincoln vehicle
 */
class Gear67 : public ::apollo::drivers::canbus::ProtocolData<
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
   * @brief get the gear state from byte array
   * config detail: {'name': 'state', 'offset': 0.0, 'precision': 1.0, 'len': 3,
   * 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of the gear state
   */
  int32_t gear_state(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief check canbus fault from byte array
   * config detail: {'name': 'fltbus', 'offset': 0.0, 'precision': 1.0, 'len':
   * 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 7, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the boolean value of canbus fault
   */
  bool is_canbus_fault(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief check driver override from byte array
   * config detail: {'name': 'driver', 'offset': 0.0, 'precision': 1.0, 'len':
   * 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 3, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the boolean value of driver override
   */
  bool is_driver_override(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief get reported gear command from byte array
   * config detail: {'name': 'cmd', 'offset': 0.0, 'precision': 1.0, 'len': 3,
   * 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 4, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of reported gear command
   */
  int32_t reported_gear_cmd(const std::uint8_t *bytes, int32_t length) const;
};

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
