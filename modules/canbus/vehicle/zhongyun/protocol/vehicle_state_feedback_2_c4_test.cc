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

#include "modules/canbus/vehicle/zhongyun/protocol/vehicle_state_feedback_2_c4.h"
#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace zhongyun {

class Vehiclestatefeedback2c4Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Vehiclestatefeedback2c4Test, reset) {
  Vehiclestatefeedback2c4 feedback_;
  int32_t length = 8;
  ChassisDetail cd;
  uint8_t bytes[8] = {0x88, 0x44, 0x22, 0x11, 0x11, 0x12, 0x13, 0x14};

  feedback_.Parse(bytes, length, &cd);
  auto &feedbackinfo = cd.zhongyun().vehicle_state_feedback_2_c4();
  EXPECT_DOUBLE_EQ(feedbackinfo.motor_speed(), 17544);
  EXPECT_DOUBLE_EQ(feedbackinfo.driven_torque_feedback(), 219.3);
}

}  // namespace zhongyun
}  // namespace canbus
}  // namespace apollo
