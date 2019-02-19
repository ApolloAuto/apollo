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
 * @file gyro_6c.h
 * @brief the class of Gyro6c (for lincoln vehicle)
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
 * @class Gyro6c
 *
 * @brief one of the protocol data of lincoln vehicle
 */
class Gyro6c : public ::apollo::drivers::canbus::ProtocolData<
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
   * @brief calculate the roll rate based on byte array.
   * config detail: {'name': 'roll', 'offset': 0.0, 'precision': 0.0002, 'len':
   * 16, 'f_type': 'value', 'is_signed_var': True, 'physical_range': '[0|0]',
   * 'bit': 0, 'type': 'double', 'order': 'intel', 'physical_unit': '"rad/s"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of roll rate
   */
  double roll_rate(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief calculate longitudinal_acceleration based on byte array.
   * config detail: {'name': 'yaw', 'offset': 0.0, 'precision': 0.0002, 'len':
   * 16, 'f_type': 'value', 'is_signed_var': True, 'physical_range': '[0|0]',
   * 'bit': 16, 'type': 'double', 'order': 'intel', 'physical_unit': '"rad/s"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of yaw rate
   */
  double yaw_rate(const std::uint8_t *bytes, int32_t length) const;
};

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
