/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/proto/chassis_detail.pb.h"

#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace minibus {

class Parkingmodefeedback18ff8dac
    : public ::apollo::drivers::canbus::ProtocolData<
          ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Parkingmodefeedback18ff8dac();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  Parkingmode_feedback_18ff8dac::Pmf_current_statusType pmf_current_status(
      const std::uint8_t* bytes, const int32_t length) const;

  Parkingmode_feedback_18ff8dac::Pmf_epb_onoffType pmf_epb_onoff(
      const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
