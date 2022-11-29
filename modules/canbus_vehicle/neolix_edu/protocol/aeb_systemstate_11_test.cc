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

#include "modules/canbus_vehicle/neolix_edu/protocol/aeb_systemstate_11.h"

#include "glog/logging.h"

#include "gtest/gtest.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace neolix_edu {

class Aebsystemstate11Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Aebsystemstate11Test, reset) {
  uint8_t data[8] = {0x67, 0x62, 0x63, 0x64, 0x51, 0x52, 0x53, 0x54};
  int32_t length = 8;
  Neolix_edu cd;
  Aebsystemstate11 accel_cmd;
  accel_cmd.Parse(data, length, &cd);
  EXPECT_EQ(data[0], 0b01100111);
  EXPECT_EQ(data[1], 0b01100010);
  EXPECT_EQ(data[2], 0b01100011);
  EXPECT_EQ(data[3], 0b01100100);
  EXPECT_EQ(data[4], 0b01010001);
  EXPECT_EQ(data[5], 0b01010010);
  EXPECT_EQ(data[6], 0b01010011);
  EXPECT_EQ(data[7], 0b01010100);

  EXPECT_EQ(cd.aeb_systemstate_11().aeb_state(), 3);
  EXPECT_EQ(cd.aeb_systemstate_11().aeb_brakestate(), true);
  EXPECT_EQ(cd.aeb_systemstate_11().faultrank(), 2);
  EXPECT_EQ(cd.aeb_systemstate_11().currenttemperature(), 59);
  EXPECT_EQ(cd.aeb_systemstate_11().pas_f1_stop(), false);
  EXPECT_EQ(cd.aeb_systemstate_11().pas_f2_stop(), false);
  EXPECT_EQ(cd.aeb_systemstate_11().pas_f3_stop(), true);
  EXPECT_EQ(cd.aeb_systemstate_11().pas_f4_stop(), false);
  EXPECT_EQ(cd.aeb_systemstate_11().pas_b1_stop(), false);
  EXPECT_EQ(cd.aeb_systemstate_11().pas_b2_stop(), true);
  EXPECT_EQ(cd.aeb_systemstate_11().pas_b3_stop(), true);
  EXPECT_EQ(cd.aeb_systemstate_11().pas_b4_stop(), false);
}

}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
