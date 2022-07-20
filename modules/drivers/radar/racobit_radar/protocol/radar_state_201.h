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

class RadarState201
    : public apollo::drivers::canbus::ProtocolData<RacobitRadar> {
 public:
  static const uint32_t ID;
  RadarState201();
  void Parse(const std::uint8_t* bytes, int32_t length,
             RacobitRadar* racobit_radar) const override;

 private:
  int max_dist(const std::uint8_t* bytes, int32_t length) const;

  int radar_power(const std::uint8_t* bytes, int32_t length) const;

  RacobitRadarState_201::OutputType output_type(
      const std::uint8_t* bytes, int32_t length) const;

  RacobitRadarState_201::RcsThreshold rcs_threshold(
      const std::uint8_t* bytes, int32_t length) const;

  bool send_quality(const std::uint8_t* bytes, int32_t length) const;

  bool send_ext_info(const std::uint8_t* bytes, int32_t length) const;
};

}  // namespace racobit_radar
}  // namespace drivers
}  // namespace apollo
