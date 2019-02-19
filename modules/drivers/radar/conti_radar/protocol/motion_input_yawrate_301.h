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

#include <cmath>

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/drivers/proto/conti_radar.pb.h"

namespace apollo {
namespace drivers {
namespace conti_radar {

using apollo::drivers::ContiRadar;
using apollo::drivers::conti_radar::OutputType;
using apollo::drivers::conti_radar::RadarConf;
using apollo::drivers::conti_radar::RcsThreshold;

class MotionInputYawRate301
    : public apollo::drivers::canbus::ProtocolData<ContiRadar> {
 public:
  static const uint32_t ID;
  MotionInputYawRate301();
  ~MotionInputYawRate301();
  /**
   * @brief get the data period
   * @return the value of data period
   */
  uint32_t GetPeriod() const override;

  /**
   * @brief update the data
   * @param data a pointer to the data to be updated
   */
  void UpdateData(uint8_t* data) override;

  /**
   * @brief reset the private variables
   */
  void Reset() override;

  void SetYawRate(const float& yaw_rate);

 private:
  float yaw_rate_ = NAN;
};

}  // namespace conti_radar
}  // namespace drivers
}  // namespace apollo
