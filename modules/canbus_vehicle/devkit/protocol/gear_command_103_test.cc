/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus_vehicle/devkit/protocol/gear_command_103.h"

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace devkit {
class Gearcommand103Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Gearcommand103Test, simple) {
  Gearcommand103 gear;
  EXPECT_EQ(gear.GetPeriod(), 20 * 1000);
  uint8_t data[8] = {0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78};
  gear.UpdateData(data);
  EXPECT_EQ(data[0], 0b01110000);
  EXPECT_EQ(data[1], 0b01110011);
  EXPECT_EQ(data[2], 0b01110011);
  EXPECT_EQ(data[3], 0b01110100);
  EXPECT_EQ(data[4], 0b01110101);
  EXPECT_EQ(data[5], 0b01110110);
  EXPECT_EQ(data[6], 0b01110111);
  EXPECT_EQ(data[7], 0b01110000);
}

}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
