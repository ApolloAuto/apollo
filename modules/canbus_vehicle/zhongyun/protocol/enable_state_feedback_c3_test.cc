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

#include "modules/canbus_vehicle/zhongyun/protocol/enable_state_feedback_c3.h"
#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace zhongyun {

class Enablestatefeedbackc3Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Enablestatefeedbackc3Test, reset) {
  Enablestatefeedbackc3 feedback_;
  int32_t length = 8;
  Zhongyun cd;
  uint8_t bytes[8] = {0x01, 0x02, 0x01, 0x02, 0x01, 0x12, 0x13, 0x14};

  feedback_.Parse(bytes, length, &cd);
  auto &feedbackinfo = cd.enable_state_feedback_c3();
  EXPECT_DOUBLE_EQ(feedbackinfo.parking_enable_state(), 1);
  EXPECT_DOUBLE_EQ(feedbackinfo.steering_enable_state(), 2);
  EXPECT_DOUBLE_EQ(feedbackinfo.gear_enable_actual(), 1);
  EXPECT_DOUBLE_EQ(feedbackinfo.driven_enable_state(), 1);
  EXPECT_DOUBLE_EQ(feedbackinfo.brake_enable_state(), 2);
}

}  // namespace zhongyun
}  // namespace canbus
}  // namespace apollo
