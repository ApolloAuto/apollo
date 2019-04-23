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

#include "modules/canbus/vehicle/zhongyun/protocol/error_state_e1.h"
#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace zhongyun {

class Errorstatee1Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Errorstatee1Test, reset) {
  Errorstatee1 error_state_;
  int32_t length = 8;
  ChassisDetail cd;
  uint8_t bytes[8] = {0x01, 0x01, 0x01, 0x01, 0x01, 0x12, 0x13, 0x14};

  error_state_.Parse(bytes, length, &cd);
  auto &error_state_info = cd.zhongyun().error_state_e1();
  EXPECT_DOUBLE_EQ(error_state_info.brake_error_code(), 1);
  EXPECT_DOUBLE_EQ(error_state_info.driven_error_code(), 1);
  EXPECT_DOUBLE_EQ(error_state_info.steering_error_code(), 1);
  EXPECT_DOUBLE_EQ(error_state_info.parking_error_code(), 1);
  EXPECT_DOUBLE_EQ(error_state_info.gear_error_msg(), 1);
}

}  // namespace zhongyun
}  // namespace canbus
}  // namespace apollo
