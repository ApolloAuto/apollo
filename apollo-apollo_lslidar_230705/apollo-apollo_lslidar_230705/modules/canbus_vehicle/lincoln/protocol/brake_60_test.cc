/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus_vehicle/lincoln/protocol/brake_60.h"

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace lincoln {

class Brake60Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Brake60Test, simple) {
  Brake60 brake;
  EXPECT_EQ(brake.GetPeriod(), 10 * 1000);
  uint8_t data[8] = {0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68};
  brake.UpdateData(data);
  EXPECT_EQ(data[0], 0b00000000);
  EXPECT_EQ(data[1], 0b00000000);
  EXPECT_EQ(data[2], 0b01100010);
  EXPECT_EQ(data[3], 0b01100100);
  EXPECT_EQ(data[4], 0b01100101);
  EXPECT_EQ(data[5], 0b01100110);
  EXPECT_EQ(data[6], 0b01100111);
  EXPECT_EQ(data[7], 0b00000000);
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
