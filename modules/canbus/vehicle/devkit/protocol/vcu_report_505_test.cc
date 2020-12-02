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

#include "modules/canbus/vehicle/devkit/protocol/vcu_report_505.h"

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace devkit {
class Vcureport505Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Vcureport505Test, General) {
  uint8_t data[8] = {0x07, 0x01, 0x01, 0x02, 0x8A, 0x03, 0x04, 0x05};
  int32_t length = 8;
  ChassisDetail cd;
  Vcureport505 vcureport;
  vcureport.Parse(data, length, &cd);

  EXPECT_EQ(data[0], 0b00000111);
  EXPECT_EQ(data[1], 0b00000001);
  EXPECT_EQ(data[2], 0b00000001);
  EXPECT_EQ(data[3], 0b00000010);
  EXPECT_EQ(data[4], 0b10001010);
  EXPECT_EQ(data[5], 0b00000011);
  EXPECT_EQ(data[6], 0b00000100);
  EXPECT_EQ(data[7], 0b00000101);

  EXPECT_EQ(cd.devkit().vcu_report_505().battery_soc(), 3);
  EXPECT_EQ(cd.devkit().vcu_report_505().vehicle_mode_state(), 1);
  EXPECT_EQ(cd.devkit().vcu_report_505().frontcrash_state(), 1);
  EXPECT_EQ(cd.devkit().vcu_report_505().backcrash_state(), 0);
  EXPECT_EQ(cd.devkit().vcu_report_505().aeb_state(), 0);
  EXPECT_EQ(cd.devkit().vcu_report_505().acc(), 1.12);
  EXPECT_EQ(cd.devkit().vcu_report_505().speed(), 0.258);
}

}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
