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

#include "modules/canbus/vehicle/ch/protocol/ecu_status_2_516.h"
#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace ch {
class Ecustatus2516Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Ecustatus2516Test, General) {
  uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x01, 0x12, 0x13, 0x14};
  int32_t length = 8;
  ChassisDetail cd;
  Ecustatus2516 ecustatus;
  ecustatus.Parse(data, length, &cd);

  EXPECT_EQ(data[0], 0b00000001);
  EXPECT_EQ(data[1], 0b00000010);
  EXPECT_EQ(data[2], 0b00000011);
  EXPECT_EQ(data[3], 0b00000100);
  EXPECT_EQ(data[4], 0b00000001);
  EXPECT_EQ(data[5], 0b00010010);
  EXPECT_EQ(data[6], 0b00010011);
  EXPECT_EQ(data[7], 0b00010100);

  EXPECT_EQ(cd.ch().ecu_status_2_516().battery_remaining_capacity(), 513);
  EXPECT_DOUBLE_EQ(cd.ch().ecu_status_2_516().battery_voltage(), 102.7);
  EXPECT_DOUBLE_EQ(cd.ch().ecu_status_2_516().battery_current(),
                   460.90000000000003);
  EXPECT_EQ(cd.ch().ecu_status_2_516().battery_temperature(), 5139);
}

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
