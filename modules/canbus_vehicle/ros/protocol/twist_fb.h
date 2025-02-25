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

class TwistFb
    : public ::apollo::drivers::canbus::ProtocolData<::apollo::canbus::Ros> {
 public:
  static const int32_t ID;
  TwistFb();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Ros* chassis) const override;

 private:
  int flag_stop(const std::uint8_t* bytes, const int32_t length) const;

  double x_speed(const std::uint8_t* bytes, const int32_t length) const;

  double y_speed(const std::uint8_t* bytes, const int32_t length) const;

  double z_angle_speed(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace ros
}  // namespace canbus
}  // namespace apollo
