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

#include "modules/canbus_vehicle/devkit/protocol/park_report_504.h"

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace devkit {
class Parkreport504Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Parkreport504Test, General) {
  uint8_t data[8] = {0x04, 0x01, 0x01, 0x10, 0x90, 0x01, 0x00, 0x01};
  int32_t length = 8;
  Devkit cd;
  Parkreport504 parkreport;
  parkreport.Parse(data, length, &cd);

  EXPECT_EQ(data[0], 0b00000100);
  EXPECT_EQ(data[1], 0b00000001);
  EXPECT_EQ(data[2], 0b00000001);
  EXPECT_EQ(data[3], 0b00010000);
  EXPECT_EQ(data[4], 0b10010000);
  EXPECT_EQ(data[5], 0b00000001);
  EXPECT_EQ(data[6], 0b00000000);
  EXPECT_EQ(data[7], 0b00000001);

  EXPECT_EQ(cd.park_report_504().parking_actual(), 0);
  EXPECT_EQ(cd.park_report_504().park_flt(), 1);
}

}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
