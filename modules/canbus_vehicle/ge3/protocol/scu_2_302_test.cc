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

#include "modules/canbus_vehicle/ge3/protocol/scu_2_302.h"
#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace ge3 {

class Scu2302Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Scu2302Test, reset) {
  Scu2302 scu2302;
  int32_t length = 8;
  Ge3 chassis_detail;
  uint8_t bytes[8] = {0x4D, 0x47, 0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA};

  scu2302.Parse(bytes, length, &chassis_detail);
  EXPECT_DOUBLE_EQ(chassis_detail.scu_2_302().vin00(), 'M');  // 77
  EXPECT_DOUBLE_EQ(chassis_detail.scu_2_302().vin01(), 71);
  EXPECT_DOUBLE_EQ(chassis_detail.scu_2_302().vin02(), 255);
  EXPECT_DOUBLE_EQ(chassis_detail.scu_2_302().vin03(), 238);
  EXPECT_DOUBLE_EQ(chassis_detail.scu_2_302().vin04(), 221);
  EXPECT_DOUBLE_EQ(chassis_detail.scu_2_302().vin05(), 204);
  EXPECT_DOUBLE_EQ(chassis_detail.scu_2_302().vin06(), 187);
  EXPECT_DOUBLE_EQ(chassis_detail.scu_2_302().vin07(), 170);
}

}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
