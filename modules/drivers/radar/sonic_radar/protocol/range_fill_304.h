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

#ifndef MODULES_DRIVERS_RADAR_SONIC_RADAR_PROTOCOL_RANGE_FILL_304_H_
#define MODULES_DRIVERS_RADAR_SONIC_RADAR_PROTOCOL_RANGE_FILL_304_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/drivers/proto/sonic_radar.pb.h"

namespace apollo {
namespace drivers {
namespace sonic_radar {

using apollo::drivers::Ultrasonic;

class RangeFill304
    : public apollo::drivers::canbus::ProtocolData<Ultrasonic> {
 public:
  static const uint32_t ID;
  RangeFill304();
  ~RangeFill304();
  /**
   * @brief get the data period
   * @return the value of data period
   */
  uint32_t GetPeriod() const override;

  /**
   * @brief parse the data
   * @param data a pointer to the data to be updated
   * @param length the length of the data
   * @param ultrasonic message to be filled
   */
  void Parse(const std::uint8_t* bytes, int32_t length, Ultrasonic* ultrasonic) const override;

};

}  // namespace sonic_radar
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_RADAR_CONTI_RADAR_PROTOCOL_RADAR_CONFIG_200_H_
