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

#include "modules/drivers/radar/conti_radar/protocol/motion_input_yawrate_301.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace drivers {
namespace conti_radar {

using apollo::drivers::ContiRadar;
using apollo::drivers::canbus::Byte;

const uint32_t MotionInputYawRate301::ID = 0x301;

MotionInputYawRate301::MotionInputYawRate301() {}
MotionInputYawRate301::~MotionInputYawRate301() {}

uint32_t MotionInputYawRate301::GetPeriod() const {
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

/**
 * @brief update the data
 * @param data a pointer to the data to be updated
 */
void MotionInputYawRate301::UpdateData(uint8_t* data) {
  if (std::isnan(yaw_rate_)) {
    AWARN << "yaw_rate is nan";
    return;
  }
  // Due to radar 408 manual: max 327.68, res 0.01, unit:deg/s
  uint32_t yaw_rate_value = static_cast<uint32_t>((yaw_rate_ + 327.68) * 100);
  Byte yaw_rate_low(data);
  yaw_rate_low.set_value(
      static_cast<unsigned char>((yaw_rate_value & 0xFF00) >> 8), 0, 8);
  Byte yaw_rate_high(data + 1);
  yaw_rate_high.set_value(static_cast<unsigned char>(yaw_rate_value & 0x00FF),
                          0, 8);
}

/**
 * @brief reset the private variables
 */
void MotionInputYawRate301::Reset() { yaw_rate_ = NAN; }

void MotionInputYawRate301::SetYawRate(const float& yaw_rate) {
  yaw_rate_ = yaw_rate;
}

}  // namespace conti_radar
}  // namespace drivers
}  // namespace apollo
