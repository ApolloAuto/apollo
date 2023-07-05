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

#include "modules/drivers/radar/conti_radar/protocol/motion_input_speed_300.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace drivers {
namespace conti_radar {

using apollo::drivers::ContiRadar;
using apollo::drivers::canbus::Byte;

const uint32_t MotionInputSpeed300::ID = 0x300;

MotionInputSpeed300::MotionInputSpeed300() {}
MotionInputSpeed300::~MotionInputSpeed300() {}

uint32_t MotionInputSpeed300::GetPeriod() const {
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

/**
 * @brief update the data
 * @param data a pointer to the data to be updated
 */
void MotionInputSpeed300::UpdateData(uint8_t* data) {
  if (std::isnan(speed_)) {
    AWARN << "speed is nan";
    return;
  }
  uint32_t speed_direction = 0;
  if (fabs(speed_) < 0.02) {
    speed_direction = 0;
  } else if (speed_ < 0) {
    speed_direction = 2;
  } else {
    speed_direction = 1;
  }
  uint32_t speed_value = static_cast<uint32_t>(fabs(speed_) / 0.02);
  Byte frame_speed_direction(data);
  frame_speed_direction.set_value(
      static_cast<unsigned char>((speed_direction << 6) & 0x00C0) |
          static_cast<unsigned char>((speed_value & 0x1F00) >> 8),
      0, 8);
  Byte frame_speed(data + 1);
  frame_speed.set_value(static_cast<unsigned char>(speed_value & 0x00FF), 0, 8);
}

/**
 * @brief reset the private variables
 */
void MotionInputSpeed300::Reset() { speed_ = NAN; }

void MotionInputSpeed300::SetSpeed(const float& speed) { speed_ = speed; }

}  // namespace conti_radar
}  // namespace drivers
}  // namespace apollo
