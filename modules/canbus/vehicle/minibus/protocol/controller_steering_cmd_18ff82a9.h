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

class Controllersteeringcmd18ff82a9
    : public ::apollo::drivers::canbus::ProtocolData<
          ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Controllersteeringcmd18ff82a9();

  uint32_t GetPeriod() const override;

  void UpdateData_Heartbeat(uint8_t* data) override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  Controllersteeringcmd18ff82a9* set_steering_vcu_status(
      int steering_vcu_status);

  Controllersteeringcmd18ff82a9* set_steering_heartbeat(int steering_heartbeat);

  Controllersteeringcmd18ff82a9* set_steering_velocity(
      double steering_velocity);

  Controllersteeringcmd18ff82a9* set_steering_ctrl_status(
      Controller_steering_cmd_18ff82a9::Steering_ctrl_statusType
          steering_ctrl_status);

  Controllersteeringcmd18ff82a9* set_streering_angle(double streering_angle);

 private:
  void set_p_steering_vcu_status(uint8_t* data, int steering_vcu_status);

  void set_p_steering_heartbeat(uint8_t* data, int steering_heartbeat);

  void set_p_steering_velocity(uint8_t* data, double steering_velocity);

  void set_p_steering_ctrl_status(
      uint8_t* data, Controller_steering_cmd_18ff82a9::Steering_ctrl_statusType
                         steering_ctrl_status);

  void set_p_streering_angle(uint8_t* data, double streering_angle);

 private:
  int steering_vcu_status_;
  int steering_heartbeat_;
  double steering_velocity_;
  Controller_steering_cmd_18ff82a9::Steering_ctrl_statusType
      steering_ctrl_status_;
  double streering_angle_;
};

}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
