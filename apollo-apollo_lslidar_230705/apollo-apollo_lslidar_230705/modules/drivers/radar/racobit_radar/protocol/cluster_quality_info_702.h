/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/common_msgs/sensor_msgs/racobit_radar.pb.h"

namespace apollo {
namespace drivers {
namespace racobit_radar {

using apollo::drivers::RacobitRadar;

class ClusterQualityInfo702
    : public apollo::drivers::canbus::ProtocolData<RacobitRadar> {
 public:
  static const uint32_t ID;
  ClusterQualityInfo702();
  void Parse(const std::uint8_t* bytes, int32_t length,
             RacobitRadar* racobit_radar) const override;

 private:
  int target_id(const std::uint8_t* bytes, int32_t length) const;

  int longitude_dist_rms(const std::uint8_t* bytes, int32_t length) const;

  int lateral_dist_rms(const std::uint8_t* bytes, int32_t length) const;

  int longitude_vel_rms(const std::uint8_t* bytes, int32_t length) const;

  int pdh0(const std::uint8_t* bytes, int32_t length) const;

  int ambig_state(const std::uint8_t* bytes, int32_t length) const;

  int invalid_state(const std::uint8_t* bytes, int32_t length) const;

  int lateral_vel_rms(const std::uint8_t* bytes, int32_t length) const;
};

}  // namespace racobit_radar
}  // namespace drivers
}  // namespace apollo
