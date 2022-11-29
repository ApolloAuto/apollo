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

#include "modules/canbus_vehicle/devkit/protocol/steering_report_502.h"

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace devkit {

class Steeringreport502Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Steeringreport502Test, General) {
  uint8_t data[8] = {0x04, 0x01, 0x01, 0x14, 0x1E, 0x03, 0x04, 0x05};
  int32_t length = 8;
  Devkit cd;
  Steeringreport502 steeringreport;
  steeringreport.Parse(data, length, &cd);

  EXPECT_EQ(data[0], 0b00000100);
  EXPECT_EQ(data[1], 0b00000001);
  EXPECT_EQ(data[2], 0b00000001);
  EXPECT_EQ(data[3], 0b00010100);
  EXPECT_EQ(data[4], 0b00011110);
  EXPECT_EQ(data[5], 0b00000011);
  EXPECT_EQ(data[6], 0b00000100);
  EXPECT_EQ(data[7], 0b00000101);

  EXPECT_EQ(cd.steering_report_502().steer_angle_spd_actual(), 5);
  EXPECT_EQ(cd.steering_report_502().steer_flt2(), 1);
  EXPECT_EQ(cd.steering_report_502().steer_flt1(), 1);
  EXPECT_EQ(cd.steering_report_502().steer_en_state(), 0);
  EXPECT_EQ(cd.steering_report_502().steer_angle_actual(), 4650);
  EXPECT_EQ(cd.steering_report_502().steer_angle_rear_actual(), 272);
}

}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
