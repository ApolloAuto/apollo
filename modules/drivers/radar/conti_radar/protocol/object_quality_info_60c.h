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

#pragma once

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/drivers/proto/conti_radar.pb.h"

namespace apollo {
namespace drivers {
namespace conti_radar {

using apollo::drivers::ContiRadar;

class ObjectQualityInfo60C
    : public apollo::drivers::canbus::ProtocolData<ContiRadar> {
 public:
  static const uint32_t ID;
  ObjectQualityInfo60C();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ContiRadar* conti_radar) const override;

 private:
  int object_id(const std::uint8_t* bytes, int32_t length) const;

  int longitude_dist_rms(const std::uint8_t* bytes, int32_t length) const;

  int lateral_dist_rms(const std::uint8_t* bytes, int32_t length) const;

  int longitude_vel_rms(const std::uint8_t* bytes, int32_t length) const;

  int lateral_vel_rms(const std::uint8_t* bytes, int32_t length) const;

  int longitude_accel_rms(const std::uint8_t* bytes, int32_t length) const;

  int lateral_accel_rms(const std::uint8_t* bytes, int32_t length) const;

  int oritation_angle_rms(const std::uint8_t* bytes, int32_t length) const;

  int probexist(const std::uint8_t* bytes, int32_t length) const;

  int meas_state(const std::uint8_t* bytes, int32_t length) const;
};

}  // namespace conti_radar
}  // namespace drivers
}  // namespace apollo
