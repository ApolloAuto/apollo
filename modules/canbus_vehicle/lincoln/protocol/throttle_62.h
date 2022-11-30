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
 * @file throttle_62.h
 * @brief the class of Throttle62 (for lincoln vehicle)
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
 * @class Throttle62
 *
 * @brief one of the protocol data of lincoln vehicle
 */
class Throttle62 : public ::apollo::drivers::canbus::ProtocolData<
                       ::apollo::canbus::Lincoln> {
 public:
  static const int32_t ID;

  /**
   * @brief get the data period
   * @return the value of data period
   */
  virtual uint32_t GetPeriod() const;

  /**
   * @brief update the data
   * @param data a pointer to the data to be updated
   */
  virtual void UpdateData(uint8_t *data);

  /**
   * @brief reset the private variables
   */
  virtual void Reset();

  /**
   * @brief set pedal based on pedal command
   * @return a this pointer to the instance itself
   */
  Throttle62 *set_pedal(double pcmd);

  /**
   * @brief set enable
   * @return a this pointer to the instance itself
   */
  Throttle62 *set_enable();

  /**
   * @brief set disable
   * @return a this pointer to the instance itself
   */
  Throttle62 *set_disable();

  /**
   * @brief set enable ignore driver override
   * @return a this pointer to the instance itself
   */
  Throttle62 *set_enable_ignore_driver_override();

 private:
  /**
   * config detail: {'name': 'pcmd', 'offset': 0.0, 'precision':
   * 1.52590218966964e-05, 'len': 16, 'f_type': 'value', 'is_signed_var': False,
   * 'physical_range': '[0|1]', 'bit': 0, 'type': 'double', 'order': 'intel',
   * 'physical_unit': '"%"'}
   */
  void set_pedal_p(uint8_t *data, double pcmd);

  /**
   * config detail: {'name': 'en', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 24, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   */
  void set_enable_p(uint8_t *bytes, bool en);

  /**
   * config detail: {'name': 'clear', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 25, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   */
  void set_clear_driver_override_flag_p(uint8_t *bytes, bool clear);

  /**
   * config detail: {'name': 'ignore', 'offset': 0.0, 'precision': 1.0, 'len':
   * 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 26, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   */
  void set_ignore_driver_override_p(uint8_t *bytes, bool ignore);

  /**
   * config detail: {'name': 'count', 'offset': 0.0, 'precision': 1.0, 'len': 8,
   * 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|255]',
   * 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
   */
  void set_watchdog_counter_p(uint8_t *data, int32_t count);

 private:
  double pedal_cmd_ = 0.0;
  bool pedal_enable_ = false;
  bool clear_driver_override_flag_ = false;
  bool ignore_driver_override_ = false;
  int32_t watchdog_counter_ = 0;
};

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
