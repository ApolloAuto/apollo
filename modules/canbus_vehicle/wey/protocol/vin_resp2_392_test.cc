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

#include "modules/canbus_vehicle/wey/protocol/vin_resp2_392.h"
#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace wey {

class Vinresp2392Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Vinresp2392Test, reset) {
  Vinresp2392 vin2;
  int32_t length = 8;
  Wey chassis_detail;
  uint8_t bytes[8] = {0x88, 0x44, 0x22, 0x11, 0x11, 0x12, 0x13, 0x14};

  vin2.Parse(bytes, length, &chassis_detail);
  EXPECT_DOUBLE_EQ(chassis_detail.vin_resp2_392().vin15(), 136);
  EXPECT_DOUBLE_EQ(chassis_detail.vin_resp2_392().vin14(), 68);
  EXPECT_DOUBLE_EQ(chassis_detail.vin_resp2_392().vin13(), 34);
  EXPECT_DOUBLE_EQ(chassis_detail.vin_resp2_392().vin12(), 17);
  EXPECT_DOUBLE_EQ(chassis_detail.vin_resp2_392().vin11(), 17);
  EXPECT_DOUBLE_EQ(chassis_detail.vin_resp2_392().vin10(), 18);
  EXPECT_DOUBLE_EQ(chassis_detail.vin_resp2_392().vin09(), 19);
  EXPECT_DOUBLE_EQ(chassis_detail.vin_resp2_392().vin08(), 20);
}

}  // namespace wey
}  // namespace canbus
}  // namespace apollo
