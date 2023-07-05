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
 * @file turnsignal_68.h
 * @brief the class of Turnsignal68 (for lincoln vehicle)
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
 * @class Turnsignal68
 *
 * @brief one of the protocol data of lincoln vehicle
 */
class Turnsignal68 : public ::apollo::drivers::canbus::ProtocolData<
                         ::apollo::canbus::Lincoln> {
 public:
  static const int32_t ID;

  /**
   * @brief get the data period
   * @return the value of data period
   */
  virtual uint32_t GetPeriod() const;

  /**
   * @brief get the turn command
   * @return the turn command
   */
  virtual int32_t turn_cmd() const;

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
   * @brief set no-turn based on pedal command
   * @return a this pointer to the instance itself
   */
  Turnsignal68 *set_turn_none();

  /**
   * @brief set turn left based on pedal command
   * @return a this pointer to the instance itself
   */
  Turnsignal68 *set_turn_left();

  /**
   * @brief set turn right based on pedal command
   * @return a this pointer to the instance itself
   */
  Turnsignal68 *set_turn_right();

 private:
  /**
   * config detail: {'name': 'trncmd', 'offset': 0.0, 'precision': 1.0, 'len':
   * 2, 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
   */
  void set_turn_cmd_p(uint8_t *data, int32_t trncmd);

 private:
  int32_t turn_cmd_ = 0;
};

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
