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

#include "modules/canbus_vehicle/devkit/protocol/brake_report_501.h"

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace devkit {
class Brakereport501Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Brakereport501Test, General) {
  uint8_t data[8] = {0x01, 0x00, 0x01, 0x03, 0x52, 0x01, 0x00, 0x01};
  int32_t length = 8;
  Devkit cd;
  Brakereport501 brakereport;
  brakereport.Parse(data, length, &cd);

  EXPECT_EQ(data[0], 0b00000001);
  EXPECT_EQ(data[1], 0b00000000);
  EXPECT_EQ(data[2], 0b00000001);
  EXPECT_EQ(data[3], 0b00000011);
  EXPECT_EQ(data[4], 0b01010010);
  EXPECT_EQ(data[5], 0b00000001);
  EXPECT_EQ(data[6], 0b00000000);
  EXPECT_EQ(data[7], 0b00000001);

  EXPECT_EQ(cd.brake_report_501().brake_pedal_actual(), 85);
  EXPECT_EQ(cd.brake_report_501().brake_flt2(), 1);
  EXPECT_EQ(cd.brake_report_501().brake_flt1(), 0);
  EXPECT_EQ(cd.brake_report_501().brake_en_state(), 1);
}

}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
