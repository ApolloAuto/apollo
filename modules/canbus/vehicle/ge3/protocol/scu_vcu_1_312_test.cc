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

#include "modules/canbus/vehicle/ge3/protocol/scu_vcu_1_312.h"
#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace ge3 {

class Scuvcu1312Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Scuvcu1312Test, reset) {
  Scuvcu1312 scuvcu1312;
  int32_t length = 8;
  ChassisDetail chassis_detail;
  uint8_t bytes[8] = {0x01, 0x02, 0x03, 0x04, 0x11, 0x12, 0x13, 0x14};

  scuvcu1312.Parse(bytes, length, &chassis_detail);
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_vcu_1_312().vcu_elcsysfault(), 1);
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_vcu_1_312().vcu_brkpedst(), 1);
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_vcu_1_312().vcu_intidx(), 4);
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_vcu_1_312().vcu_gearintidx(), 2);
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_vcu_1_312().vcu_geardrvmode(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_vcu_1_312().vcu_accpedact(), 14.45);
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_vcu_1_312().vcu_brkpedpst(), 6.664);
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_vcu_1_312().vcu_vehrng(), 515);
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_vcu_1_312().vcu_accpedpst(), 1.568);
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_vcu_1_312().vcu_vehrdyst(), 1);
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_vcu_1_312().vcu_faultst(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_vcu_1_312().vcu_drvmode(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_vcu_1_312().vcu_gearpst(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_vcu_1_312().vcu_gearfaultst(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_vcu_1_312().vcu_gearact(), 0);
}

}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
