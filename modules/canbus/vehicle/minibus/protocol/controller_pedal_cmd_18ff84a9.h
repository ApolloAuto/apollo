/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include "modules/canbus/proto/chassis_detail.pb.h"

#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace minibus {

class Controllerpedalcmd18ff84a9
    : public ::apollo::drivers::canbus::ProtocolData<
          ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Controllerpedalcmd18ff84a9();

  uint32_t GetPeriod() const override;

  void UpdateData_Heartbeat(uint8_t* data) override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  Controllerpedalcmd18ff84a9* set_pedal_check(int pedal_check);

  Controllerpedalcmd18ff84a9* set_pedal_heartbeat(int pedal_heartbeat);

  Controllerpedalcmd18ff84a9* set_pedal_break(double pedal_break);

  Controllerpedalcmd18ff84a9* set_brake_select(int brake_select);

  Controllerpedalcmd18ff84a9* set_pedal_gear(
      Controller_pedal_cmd_18ff84a9::Pedal_gearType pedal_gear);

  Controllerpedalcmd18ff84a9* set_pedal_throttle(double pedal_throttle);

  Controllerpedalcmd18ff84a9* set_pedal_ctrl_request(
      Controller_pedal_cmd_18ff84a9::Pedal_ctrl_requestType pedal_ctrl_request);

 private:
  void set_p_pedal_check(uint8_t* data, int pedal_check);

  void set_p_pedal_heartbeat(uint8_t* data, int pedal_heartbeat);

  void set_p_pedal_break(uint8_t* data, double pedal_break);

  void set_p_brake_select(uint8_t* data, int brake_select);

  void set_p_pedal_gear(
      uint8_t* data, Controller_pedal_cmd_18ff84a9::Pedal_gearType pedal_gear);

  void set_p_pedal_throttle(uint8_t* data, double pedal_throttle);

  void set_p_pedal_ctrl_request(
      uint8_t* data,
      Controller_pedal_cmd_18ff84a9::Pedal_ctrl_requestType pedal_ctrl_request);

 private:
  int pedal_check_;
  int pedal_heartbeat_;
  double pedal_break_;
  int brake_select_;
  Controller_pedal_cmd_18ff84a9::Pedal_gearType pedal_gear_;
  double pedal_throttle_;
  Controller_pedal_cmd_18ff84a9::Pedal_ctrl_requestType pedal_ctrl_request_;
};

}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
