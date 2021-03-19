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

class Llcmotioncommandfeedback122
    : public ::apollo::drivers::canbus::ProtocolData<
          ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Llcmotioncommandfeedback122();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;
  FRIEND_TEST(Motioncommandfeedback1_22_test, General);

 private:
  // config detail: {'description': 'Steering angle setpoint (after limits)',
  // 'offset': 0.0, 'precision': 0.05, 'len': 16, 'name':
  // 'LLC_FBK_SteeringAngleSetpoint', 'is_signed_var': True, 'physical_range':
  // '[-1638.4|1638.35]', 'bit': 21, 'type': 'double', 'order': 'intel',
  // 'physical_unit': 'deg'}
  double llc_fbk_steeringanglesetpoint(const std::uint8_t* bytes,
                                       const int32_t length) const;

  // config detail: {'description': 'Current throttle setpoint (after limits)',
  // 'offset': 0.0, 'precision': 0.1, 'len': 10, 'name':
  // 'LLC_FBK_ThrottleSetpoint', 'is_signed_var': False, 'physical_range':
  // '[0|102.3]', 'bit': 11, 'type': 'double', 'order': 'intel',
  // 'physical_unit': '%'}
  double llc_fbk_throttlesetpoint(const std::uint8_t* bytes,
                                  const int32_t length) const;

  // config detail: {'description': 'Front brake pressure setpoint (after
  // limits)', 'offset': 0.0, 'precision': 0.0556, 'len': 11, 'name':
  // 'LLC_FBK_BrakePercentSetpoint', 'is_signed_var': False, 'physical_range':
  // '[0|113.8132]', 'bit': 0, 'type': 'double', 'order': 'intel',
  // 'physical_unit': '%'}
  double llc_fbk_brakepercentsetpoint(const std::uint8_t* bytes,
                                      const int32_t length) const;

  // config detail: {'description': 'Motion command feedback 2 heartbeat
  // counter', 'offset': 0.0, 'precision': 1.0, 'len': 2, 'name':
  // 'LLC_MotionCommandFeedback1_Count', 'is_signed_var': False,
  // 'physical_range': '[0|3]', 'bit': 54, 'type': 'int', 'order': 'intel',
  // 'physical_unit': ''}
  int llc_motioncommandfeedback1_count(const std::uint8_t* bytes,
                                       const int32_t length) const;

  // config detail: {'description': 'Motion command feedback 1 checksum',
  // 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name':
  // 'LLC_MotionCommandFeedback1_Check', 'is_signed_var': False,
  // 'physical_range': '[0|255]', 'bit': 56, 'type': 'int', 'order': 'intel',
  // 'physical_unit': ''}
  int llc_motioncommandfeedback1_check(const std::uint8_t* bytes,
                                       const int32_t length) const;
};

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
