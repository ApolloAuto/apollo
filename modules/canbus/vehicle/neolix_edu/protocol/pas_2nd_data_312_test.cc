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

#include "modules/canbus/vehicle/neolix_edu/protocol/pas_2nd_data_312.h"

#include "glog/logging.h"

#include "gtest/gtest.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace neolix_edu {

class Pas2nddata312Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Pas2nddata312Test, reset) {
  uint8_t data[8] = {0x67, 0x62, 0x63, 0x64, 0x51, 0x52, 0x53, 0x54};
  int32_t length = 8;
  ChassisDetail cd;
  Pas2nddata312 accel_cmd;
  accel_cmd.Parse(data, length, &cd);
  EXPECT_EQ(data[0], 0b01100111);
  EXPECT_EQ(data[1], 0b01100010);
  EXPECT_EQ(data[2], 0b01100011);
  EXPECT_EQ(data[3], 0b01100100);
  EXPECT_EQ(data[4], 0b01010001);
  EXPECT_EQ(data[5], 0b01010010);
  EXPECT_EQ(data[6], 0b01010011);
  EXPECT_EQ(data[7], 0b01010100);

  EXPECT_EQ(cd.neolix_edu().pas_2nd_data_312().pas_b1_status(), true);
  EXPECT_EQ(cd.neolix_edu().pas_2nd_data_312().pas_b2_status(), true);
  EXPECT_EQ(cd.neolix_edu().pas_2nd_data_312().pas_b3_status(), true);
  EXPECT_EQ(cd.neolix_edu().pas_2nd_data_312().pas_b4_status(), false);
  EXPECT_EQ(cd.neolix_edu().pas_2nd_data_312().pasdistance1(), 196);
  EXPECT_EQ(cd.neolix_edu().pas_2nd_data_312().pasdistance2(), 198);
  EXPECT_EQ(cd.neolix_edu().pas_2nd_data_312().pasdistance3(), 200);
  EXPECT_EQ(cd.neolix_edu().pas_2nd_data_312().pasdistance4(), 162);
}

}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
