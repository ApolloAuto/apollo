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

#include "modules/canbus_vehicle/wey/protocol/fail_241.h"
#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace wey {

class Fail241Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Fail241Test, reset) {
  Fail241 fail1;
  int32_t length = 8;
  Wey chassis_detail;
  uint8_t bytes[8] = {0x88, 0x44, 0x22, 0x11, 0x95, 0x12, 0x13, 0x14};

  fail1.Parse(bytes, length, &chassis_detail);
  EXPECT_DOUBLE_EQ(chassis_detail.fail_241().engfail(), 1);
  EXPECT_DOUBLE_EQ(chassis_detail.fail_241().espfail(), 1);
  EXPECT_DOUBLE_EQ(chassis_detail.fail_241().epbfail(), 1);
  EXPECT_DOUBLE_EQ(chassis_detail.fail_241().shiftfail(), 1);
  EXPECT_DOUBLE_EQ(chassis_detail.fail_241().epsfail(), 1);
}

}  // namespace wey
}  // namespace canbus
}  // namespace apollo
