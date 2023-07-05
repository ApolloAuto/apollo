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
 * @file accel_6b.h
 * @brief the class of Accel6b (for lincoln vehicle)
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
 * @class Accel6b
 *
 * @brief one of the protocol data of lincoln vehicle
 */
class Accel6b : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::Lincoln> {
 public:
  static const int32_t ID;

  /*
   * @brief parse received data
   * @param bytes a pointer to the input bytes
   * @param length the length of the input bytes
   * @param chassis_detail the parsed chassis_detail
   */
  virtual void Parse(const std::uint8_t *bytes, int32_t length,
                     Lincoln *chassis_detail) const;

 private:
  /**
   * @brief calculate lateral acceleration based on byte array.
   * Config detail: {'name': 'lat', 'offset': 0.0, 'precision': 0.01, 'len': 16,
   * 'f_type': 'value', 'is_signed_var': True, 'physical_range': '[0|0]', 'bit':
   * 0, 'type': 'double', 'order': 'intel', 'physical_unit': '"m/s^2"'}
   * @param bytes a pointer to the byte array
   * @return the value of lateral acceleration
   */
  double lateral_acceleration(const std::uint8_t *bytes,
                              const int32_t length) const;

  /**
   * @brief calculate longitudinal_acceleration based on byte array.
   * Config detail: {'name': 'long', 'offset': 0.0, 'precision': 0.01, 'len':
   * 16, 'f_type': 'value', 'is_signed_var': True, 'physical_range': '[0|0]',
   * 'bit': 16, 'type': 'double', 'order': 'intel', 'physical_unit': '"m/s^2"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of longitudinal acceleration
   */
  double longitudinal_acceleration(const std::uint8_t *bytes,
                                   const int32_t length) const;
  /**
   * @brief calculate vertical_acceleration based on byte array.
   * Config detail: {'name': 'vert', 'offset': 0.0, 'precision': 0.01, 'len':
   * 16, 'f_type': 'value', 'is_signed_var': True, 'physical_range': '[0|0]',
   * 'bit': 32, 'type': 'double', 'order': 'intel', 'physical_unit': '"m/s^2"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of vertical acceleration
   */
  double vertical_acceleration(const std::uint8_t *bytes,
                               const int32_t length) const;

  double parse_two_frames(const std::uint8_t low_byte,
                          const std::uint8_t high_byte) const;
};

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
