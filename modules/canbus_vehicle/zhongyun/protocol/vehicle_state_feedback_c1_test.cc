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

#include "modules/canbus_vehicle/zhongyun/protocol/vehicle_state_feedback_c1.h"
#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace zhongyun {

class Vehiclestatefeedbackc1Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Vehiclestatefeedbackc1Test, reset) {
  Vehiclestatefeedbackc1 feedback_;
  int32_t length = 8;
  Zhongyun cd;
  uint8_t bytes[8] = {0x88, 0x44, 0x22, 0x11, 0x03, 0x12, 0x13, 0x01};

  feedback_.Parse(bytes, length, &cd);

  auto &feedbackinfo = cd.vehicle_state_feedback_c1();
  EXPECT_DOUBLE_EQ(feedbackinfo.parking_actual(), 1);
  EXPECT_DOUBLE_EQ(feedbackinfo.brake_torque_feedback(), 244.1);
  EXPECT_DOUBLE_EQ(feedbackinfo.gear_state_actual(), 3);
  EXPECT_DOUBLE_EQ(feedbackinfo.steering_actual(), -1419.05);
  EXPECT_DOUBLE_EQ(feedbackinfo.speed(), 48.733333333333334);
}

}  // namespace zhongyun
}  // namespace canbus
}  // namespace apollo
