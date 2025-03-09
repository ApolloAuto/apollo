// Copyright 2025 WheelOS. All Rights Reserved.
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

#include "modules/serial/vehicle/ros/ros_parser.h"

#include "cyber/logger/log.h"
#include "modules/serial/vehicle/ros/protocol/twist_cmd.h"

namespace apollo {
namespace serial {

bool ROSParser::Encode(const ControlCommand& cmd, uint8_t* data,
                       size_t length) {
  set_x_target_speed(data, cmd.speed());
  set_angular_velocity_z(data, cmd.steering_rate());

  set_checksum(data);
  return true;
}

bool ROSParser::DecodeTwistFb(const uint8_t* data, size_t length,
                              Chassis* chassis) {
  chassis->set_speed_mps(x_speed(data, length));
  return true;
}

bool ROSParser::DecodeMiscFb(const uint8_t* data, size_t length,
                             Chassis* chassis) {
  chassis->set_battery_soc_percentage(battery_voltage(data, length));
  return true;
}

}  // namespace serial
}  // namespace apollo
