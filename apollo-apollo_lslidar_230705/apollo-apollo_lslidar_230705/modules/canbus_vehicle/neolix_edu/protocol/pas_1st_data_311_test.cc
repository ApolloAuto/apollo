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

#include "modules/canbus_vehicle/neolix_edu/protocol/pas_1st_data_311.h"

#include "glog/logging.h"

#include "gtest/gtest.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace neolix_edu {

class Pas1stdata311Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Pas1stdata311Test, reset) {
  uint8_t data[8] = {0x67, 0x62, 0x63, 0x64, 0x51, 0x52, 0x53, 0x54};
  int32_t length = 8;
  Neolix_edu cd;
  Pas1stdata311 accel_cmd;
  accel_cmd.Parse(data, length, &cd);
  EXPECT_EQ(data[0], 0b01100111);
  EXPECT_EQ(data[1], 0b01100010);
  EXPECT_EQ(data[2], 0b01100011);
  EXPECT_EQ(data[3], 0b01100100);
  EXPECT_EQ(data[4], 0b01010001);
  EXPECT_EQ(data[5], 0b01010010);
  EXPECT_EQ(data[6], 0b01010011);
  EXPECT_EQ(data[7], 0b01010100);

  EXPECT_EQ(cd.pas_1st_data_311().pasdistance4(), 162);
  EXPECT_EQ(cd.pas_1st_data_311().pasdistance3(), 200);
  EXPECT_EQ(cd.pas_1st_data_311().pas_f1_status(), true);
  EXPECT_EQ(cd.pas_1st_data_311().pas_f2_status(), true);
  EXPECT_EQ(cd.pas_1st_data_311().pas_f3_status(), true);
  EXPECT_EQ(cd.pas_1st_data_311().pas_f4_status(), false);
  EXPECT_EQ(cd.pas_1st_data_311().pasdistance2(), 198);
  EXPECT_EQ(cd.pas_1st_data_311().pasdistance1(), 196);
}

}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
