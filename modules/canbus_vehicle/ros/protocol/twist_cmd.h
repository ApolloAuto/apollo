// Copyright 2025 daohu527@gmail.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//  Created Date: 2025-01-16
//  Author: daohu527

#pragma once

#include "modules/canbus_vehicle/ros/proto/ros.pb.h"

#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace ros {

class TwistCmd
    : public ::apollo::drivers::canbus::ProtocolData<::apollo::canbus::Ros> {
 public:
  static const int32_t ID;

  TwistCmd();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  void set_x_target_speed(double x_target_speed);

  void set_y_target_speed(double y_target_speed);

  void set_angular_velocity_z(double z_angular_velocity);

 private:
  void set_p_x_target_speed(uint8_t* data, double x_target_speed);

  void set_p_y_target_speed(uint8_t* data, double y_target_speed);

  void set_p_angular_velocity_z(uint8_t* data, double z_angular_velocity);

  void set_checksum(uint8_t* data, uint8_t checksum);

 private:
  uint32_t x_target_speed_;
  uint32_t y_target_speed_;
  int z_angular_velocity_;
};

}  // namespace ros
}  // namespace canbus
}  // namespace apollo
