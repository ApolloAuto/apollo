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

class Epsfeedback18ff83aa : public ::apollo::drivers::canbus::ProtocolData<
                                ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Epsfeedback18ff83aa();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  Eps_feedback_18ff83aa::Eps_vcu_statusType eps_vcu_status(
      const std::uint8_t* bytes, const int32_t length) const;

  double eps_target_steering_velocity(const std::uint8_t* bytes,
                                      const int32_t length) const;

  double eps_real_angle(const std::uint8_t* bytes, const int32_t length) const;

  double eps_targit_angle(const std::uint8_t* bytes,
                          const int32_t length) const;

  double eps_wheel_torque(const std::uint8_t* bytes,
                          const int32_t length) const;
};

}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
