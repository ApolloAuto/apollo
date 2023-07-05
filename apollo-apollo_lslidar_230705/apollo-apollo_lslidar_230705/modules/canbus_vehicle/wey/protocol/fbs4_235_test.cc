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

#include "modules/canbus_vehicle/wey/protocol/fbs4_235.h"
#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace wey {

class Fbs4235Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Fbs4235Test, reset) {
  Fbs4235 fbs4;
  int32_t length = 8;
  Wey chassis_detail;
  uint8_t bytes[8] = {0x04, 0x03, 0x02, 0x01, 0x11, 0x12, 0x13, 0x14};

  fbs4.Parse(bytes, length, &chassis_detail);
  EXPECT_DOUBLE_EQ(chassis_detail.fbs4_235().steerwheelangle(), 38.5);
  EXPECT_DOUBLE_EQ(chassis_detail.fbs4_235().steerwheelspd(), 218.5);
}

}  // namespace wey
}  // namespace canbus
}  // namespace apollo
