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

#include "modules/canbus_vehicle/wey/protocol/fbs2_240.h"
#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace wey {

class Fbs2240Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Fbs2240Test, reset) {
  Fbs2240 fbs2;
  int32_t length = 8;
  Wey chassis_detail;
  uint8_t bytes[8] = {0x88, 0x44, 0x22, 0x11, 0x11, 0x12, 0x13, 0x14};

  fbs2.Parse(bytes, length, &chassis_detail);
  EXPECT_DOUBLE_EQ(chassis_detail.fbs2_240().flwheeldirection(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.fbs2_240().frwheelspd(), 245.25);
  EXPECT_DOUBLE_EQ(chassis_detail.fbs2_240().rlwheeldrivedirection(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.fbs2_240().rlwheelspd(), 61.3125);
  EXPECT_DOUBLE_EQ(chassis_detail.fbs2_240().rrwheeldirection(), 1);
  EXPECT_DOUBLE_EQ(chassis_detail.fbs2_240().rrwheelspd(), 30.7125);
}

}  // namespace wey
}  // namespace canbus
}  // namespace apollo
