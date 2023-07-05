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

#include "modules/canbus_vehicle/devkit/protocol/gear_report_503.h"

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace devkit {
class Gearreport503Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Gearreport503Test, General) {
  uint8_t data[8] = {0x04, 0x01, 0x01, 0x10, 0x90, 0x01, 0x00, 0x01};
  int32_t length = 8;
  Devkit cd;
  Gearreport503 gearreport;
  gearreport.Parse(data, length, &cd);

  EXPECT_EQ(data[0], 0b00000100);
  EXPECT_EQ(data[1], 0b00000001);
  EXPECT_EQ(data[2], 0b00000001);
  EXPECT_EQ(data[3], 0b00010000);
  EXPECT_EQ(data[4], 0b10010000);
  EXPECT_EQ(data[5], 0b00000001);
  EXPECT_EQ(data[6], 0b00000000);
  EXPECT_EQ(data[7], 0b00000001);

  EXPECT_EQ(cd.gear_report_503().gear_flt(), 1);
  EXPECT_EQ(cd.gear_report_503().gear_actual(), 4);
}

}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
