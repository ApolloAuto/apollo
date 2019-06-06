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

#include "modules/canbus/vehicle/ch/protocol/gear_command_114.h"
#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace ch {
class Gearcommand114Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Gearcommand114Test, simple) {
  Gearcommand114 gearcommand;
  EXPECT_EQ(gearcommand.GetPeriod(), 20 * 1000);
  uint8_t data[8] = {0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68};
  gearcommand.UpdateData(data);
  EXPECT_EQ(data[0], 0b00000011);
  EXPECT_EQ(data[1], 0b01100010);
  EXPECT_EQ(data[2], 0b01100011);
  EXPECT_EQ(data[3], 0b01100100);
  EXPECT_EQ(data[4], 0b01100101);
  EXPECT_EQ(data[5], 0b01100110);
  EXPECT_EQ(data[6], 0b01100111);
  EXPECT_EQ(data[7], 0b01101000);
}

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
