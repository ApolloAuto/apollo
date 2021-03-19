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
 * @file steering_64.h
 * @brief the class of Steering64 (for lincoln vehicle)
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
 * @class Steering64
 *
 * @brief one of the protocol data of lincoln vehicle
 */
class Steering64 : public ::apollo::drivers::canbus::ProtocolData<
                       ::apollo::canbus::ChassisDetail> {
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
   * @brief set steering request enable to true
   * @return a this pointer to the instance itself
   */
  Steering64 *set_enable();

  /**
   * @brief set steering request disable to true
   * @return a this pointer to the instance itself
   */
  Steering64 *set_disable();

  /**
   * @brief set steering angle
   * @return a this pointer to the instance itself
   */
  Steering64 *set_steering_angle(double angle);

  /**
   * @brief set steering angle speed
   * @return a this pointer to the instance itself
   */
  Steering64 *set_steering_angle_speed(double angle_speed);

 private:
  /**
   * config detail: {'name': 'scmd', 'offset': 0.0, 'precision': 0.1, 'len': 16,
   * 'f_type': 'value', 'is_signed_var': True, 'physical_range': '[-470|470]',
   * 'bit': 0, 'type': 'double', 'order': 'intel', 'physical_unit': '"degrees"'}
   */
  void set_steering_angle_p(uint8_t *data, double angle);

  /**
   * config detail: {'name': 'en', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 16, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   */
  void set_enable_p(uint8_t *bytes, bool enable);

  /**
   * config detail: {'name': 'clear', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 17, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   */
  void set_clear_driver_override_flag_p(uint8_t *bytes, bool clear);

  /**
   * config detail: {'name': 'ignore', 'offset': 0.0, 'precision': 1.0, 'len':
   * 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 18, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   */
  void set_ignore_driver_override_p(uint8_t *bytes, bool ignore);

  /**
   * config detail: {'name': 'svel', 'offset': 0.0, 'precision': 2.0, 'len': 8,
   * 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|500]',
   * 'bit': 24, 'type': 'double', 'order': 'intel', 'physical_unit':
   * '"degrees/s"'}
   */
  void set_steering_angle_speed_p(uint8_t *data, double angle_speed);

  /**
   * config detail: {'name': 'count', 'offset': 0.0, 'precision': 1.0, 'len': 8,
   * 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|255]',
   * 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
   */
  void set_watchdog_counter_p(uint8_t *data, int32_t count);

  void set_disable_audible_warning_p(uint8_t *data, bool disable);

 private:
  double steering_angle_ = 0.0;
  bool steering_enable_ = false;
  bool clear_driver_override_flag_ = false;
  bool ignore_driver_override_ = false;
  double steering_angle_speed_ = 0.0;
  int32_t watchdog_counter_ = 0;
  bool disable_audible_warning_ = false;
};

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
