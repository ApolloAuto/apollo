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

#include "modules/canbus_vehicle/ch/protocol/ecu_status_1_515.h"
#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace ch {
class Ecustatus1515Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Ecustatus1515Test, General) {
  uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x01, 0x12, 0x12, 0x14};
  int32_t length = 8;
  Ch cd;
  Ecustatus1515 ecustatus;
  ecustatus.Parse(data, length, &cd);

  EXPECT_EQ(data[0], 0b00000001);
  EXPECT_EQ(data[1], 0b00000010);
  EXPECT_EQ(data[2], 0b00000011);
  EXPECT_EQ(data[3], 0b00000100);
  EXPECT_EQ(data[4], 0b00000001);
  EXPECT_EQ(data[5], 0b00010010);
  EXPECT_EQ(data[6], 0b00010010);
  EXPECT_EQ(data[7], 0b00010100);

  EXPECT_DOUBLE_EQ(cd.ecu_status_1_515().speed(), 5.1299999999999999);
  EXPECT_DOUBLE_EQ(cd.ecu_status_1_515().acc_speed(), 1.0269999999999999);
  EXPECT_EQ(cd.ecu_status_1_515().ctrl_sts(), 1);
  EXPECT_EQ(cd.ecu_status_1_515().chassis_sts(), 18);
  EXPECT_EQ(cd.ecu_status_1_515().chassis_err(), 5138);
  EXPECT_EQ(cd.ecu_status_1_515().chassis_mcu_err(), 0);
  EXPECT_EQ(cd.ecu_status_1_515().chassis_mcu_can(), 0);
  EXPECT_EQ(cd.ecu_status_1_515().chassis_hw_lost(), 0);
  EXPECT_EQ(cd.ecu_status_1_515().chassis_eps_err(), 0);
  EXPECT_EQ(cd.ecu_status_1_515().chassis_eps_can(), 0);
  EXPECT_EQ(cd.ecu_status_1_515().chassis_ehb_err(), 1);
  EXPECT_EQ(cd.ecu_status_1_515().chassis_ehb_can(), 0);
  EXPECT_EQ(cd.ecu_status_1_515().chassis_bms_can(), 0);
  EXPECT_EQ(cd.ecu_status_1_515().chassis_ads_err(), 2);
}

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
