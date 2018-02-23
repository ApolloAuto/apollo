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

#ifndef MODULES_DRIVERS_RADAR_CONTI_RADAR_PROTOCOL_OBJECT_EXTENDED_INFO_60D_H_
#define MODULES_DRIVERS_RADAR_CONTI_RADAR_PROTOCOL_OBJECT_EXTENDED_INFO_60D_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/drivers/proto/conti_radar.pb.h"

namespace apollo {
namespace drivers {
namespace conti_radar {

using apollo::drivers::ContiRadar;

class ObjectExtendedInfo60D
    : public apollo::drivers::canbus::ProtocolData<ContiRadar> {
 public:
  static const uint32_t ID;
  ObjectExtendedInfo60D();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ContiRadar* conti_radar) const override;

 private:
  int object_id(const std::uint8_t* bytes, int32_t length) const;

  double longitude_accel(const std::uint8_t* bytes, int32_t length) const;

  double lateral_accel(const std::uint8_t* bytes, int32_t length) const;

  int obstacle_class(const std::uint8_t* bytes, int32_t length) const;

  double oritation_angle(const std::uint8_t* bytes, int32_t length) const;

  double object_length(const std::uint8_t* bytes, int32_t length) const;

  double object_width(const std::uint8_t* bytes, int32_t length) const;
};

}  // namespace conti_radar
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_ESR_PROTOCOL_OBJECT_EXTENDED_INFO_60D_H_
