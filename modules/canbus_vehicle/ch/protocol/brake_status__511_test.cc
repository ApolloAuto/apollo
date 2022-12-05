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

#include "modules/canbus_vehicle/ch/protocol/brake_status__511.h"
#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace ch {
class Brakestatus511Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Brakestatus511Test, General) {
  uint8_t data[8] = {0x01, 0x02, 0x01, 0x00, 0x01, 0x01, 0x00, 0x01};
  int32_t length = 8;
  Ch cd;
  Brakestatus511 brake;
  brake.Parse(data, length, &cd);

  EXPECT_EQ(data[0], 0b00000001);
  EXPECT_EQ(data[1], 0b00000010);
  EXPECT_EQ(data[2], 0b00000001);
  EXPECT_EQ(data[3], 0b00000000);
  EXPECT_EQ(data[4], 0b00000001);
  EXPECT_EQ(data[5], 0b00000001);
  EXPECT_EQ(data[6], 0b00000000);
  EXPECT_EQ(data[7], 0b00000001);

  EXPECT_EQ(cd.brake_status__511().brake_pedal_en_sts(), 1);
  EXPECT_EQ(cd.brake_status__511().brake_pedal_sts(), 2);
  EXPECT_EQ(cd.brake_status__511().brake_err(), 1);
  EXPECT_EQ(cd.brake_status__511().emergency_btn_env(), 0);
  EXPECT_EQ(cd.brake_status__511().front_bump_env(), 1);
  EXPECT_EQ(cd.brake_status__511().back_bump_env(), 1);
  EXPECT_EQ(cd.brake_status__511().overspd_env(), 0);
}

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
