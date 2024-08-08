/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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

#include "modules/common_msgs/sensor_msgs/nano_radar.pb.h"

#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace drivers {
namespace nano_radar {

using apollo::drivers::NanoRadar;

class ObjectGeneralInfo60B
    : public apollo::drivers::canbus::ProtocolData<NanoRadar> {
 public:
  static const uint32_t ID;
  ObjectGeneralInfo60B();
  void Parse(const std::uint8_t* bytes, int32_t length,
             NanoRadar* nano_radar) const override;

 private:
  int object_id(const std::uint8_t* bytes, int32_t length) const;

  double longitude_dist(const std::uint8_t* bytes, int32_t length) const;

  double lateral_dist(const std::uint8_t* bytes, int32_t length) const;

  double longitude_vel(const std::uint8_t* bytes, int32_t length) const;

  double lateral_vel(const std::uint8_t* bytes, int32_t length) const;

  double rcs(const std::uint8_t* bytes, int32_t length) const;

  int dynprop(const std::uint8_t* bytes, int32_t length) const;

  double obstacle_range(double longitude_dist, double lateral_dist) const;

  double obstacle_velocity(double longitude_vel, double lateral_vel,
                           double object_angel) const;

  double obstacle_angle(double longitude_dist, double lateral_dist) const;
};

}  // namespace nano_radar
}  // namespace drivers
}  // namespace apollo
