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

#ifndef MODULES_CANBUS_VEHICL_LINCOLN_PROTOCOL_WHEELSPEED_6A_H_
#define MODULES_CANBUS_VEHICL_LINCOLN_PROTOCOL_WHEELSPEED_6A_H_

#include "modules/canbus/vehicle/protocol_data.h"

/**
 * @namespace apollo::canbus::lincoln
 * @brief apollo::canbus::lincoln
 */
namespace apollo {
namespace canbus {
namespace lincoln {

/**
 * @class Wheelspeed6a
 *
 * @brief one of the protocol data of lincoln vehicle
 */
class Wheelspeed6a : public ProtocolData {
 public:
  static const int32_t ID;

  /*
   * @brief parse received data
   * @param bytes a pointer to the input bytes
   * @param length the length of the input bytes
   * @param car_status the parsed car_status
   */
  virtual void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* car_status) const;

  /*
   * @brief parse received data
   * @param bytes a pointer to the input bytes
   * @param length the length of the input bytes
   * @param timestamp the timestamp of input bytes
   * @param car_status the parsed car_status
   */
  virtual void Parse(const std::uint8_t* bytes, int32_t length,
                     const struct timeval& timestamp,
                     ChassisDetail* car_status) const;

  /**
   * @brief calculate front left wheel speed based on byte array.
   * config detail: {'name': 'fl', 'offset': 0.0, 'precision': 0.01, 'len': 16,
   * 'f_type': 'value', 'is_signed_var': True, 'physical_range': '[0|0]', 'bit':
   * 0, 'type': 'double', 'order': 'intel', 'physical_unit': '"rad/s"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of byte input
   */
  double front_left_wheel_speed(const std::uint8_t* bytes,
                                int32_t length) const;

  /**
   * @brief calculate front right wheel speed based on byte array.
   * config detail: {'name': 'fr', 'offset': 0.0, 'precision': 0.01, 'len': 16,
   * 'f_type': 'value', 'is_signed_var': True, 'physical_range': '[0|0]', 'bit':
   * 16, 'type': 'double', 'order': 'intel', 'physical_unit': '"rad/s"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of front right wheel speed
   */
  double front_right_wheel_speed(const std::uint8_t* bytes,
                                 int32_t length) const;

  /**
   * @brief calculate rear left wheel speed based on byte array.
   * config detail: {'name': 'rl', 'offset': 0.0, 'precision': 0.01, 'len': 16,
   * 'f_type': 'value', 'is_signed_var': True, 'physical_range': '[0|0]', 'bit':
   * 32, 'type': 'double', 'order': 'intel', 'physical_unit': '"rad/s"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of rear left wheel speed
   */
  double rear_left_wheel_speed(const std::uint8_t* bytes, int32_t length) const;

  /**
   * @brief calculate rear right wheel speed based on byte array.
   * config detail: {'name': 'rr', 'offset': 0.0, 'precision': 0.01, 'len': 16,
   * 'f_type': 'value', 'is_signed_var': True, 'physical_range': '[0|0]', 'bit':
   * 48, 'type': 'double', 'order': 'intel', 'physical_unit': '"rad/s"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of rear right wheel speed
   */
  double rear_right_wheel_speed(const std::uint8_t* bytes,
                                int32_t length) const;
};

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_LINCOLN_PROTOCOL_WHEELSPEED_6A_H_
