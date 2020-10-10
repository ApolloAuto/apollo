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

#include "gtest/gtest_prod.h"
#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace transit {

class Llcvehiclelimits24 : public ::apollo::drivers::canbus::ProtocolData<
                               ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Llcvehiclelimits24();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;
  FRIEND_TEST(Vehiclelimits_24_test, General);

 private:
  // config detail: {'description': 'Steering angle feedback', 'offset': 0.0,
  // 'precision': 1.0, 'len': 12, 'name': 'LLC_FBK_MaxSteeringAngle',
  // 'is_signed_var': False, 'physical_range': '[0|4095]', 'bit': 12, 'type':
  // 'int', 'order': 'intel', 'physical_unit': 'deg'}
  int llc_fbk_maxsteeringangle(const std::uint8_t* bytes,
                               const int32_t length) const;

  // config detail: {'description': 'Front brake pressure feedback', 'offset':
  // 0.0, 'precision': 1.0, 'len': 12, 'name': 'LLC_FBK_MaxBrakePercent',
  // 'is_signed_var': False, 'physical_range': '[0|113.8132]', 'bit': 0, 'type':
  // 'double', 'order': 'intel', 'physical_unit': '%'}
  double llc_fbk_maxbrakepercent(const std::uint8_t* bytes,
                                 const int32_t length) const;
};

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
