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

#include "modules/canbus_vehicle/ge3/protocol/scu_bcs_1_306.h"
#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace ge3 {

class Scubcs1306Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Scubcs1306Test, reset) {
  Scubcs1306 scubcs1306;
  int32_t length = 8;
  Ge3 chassis_detail;
  uint8_t bytes[8] = {0x01, 0x02, 0x03, 0x04, 0x11, 0x12, 0x13, 0x14};

  scubcs1306.Parse(bytes, length, &chassis_detail);
  EXPECT_DOUBLE_EQ(chassis_detail.scu_bcs_1_306().bcs_aebavailable(),
                   1);  //
  EXPECT_DOUBLE_EQ(chassis_detail.scu_bcs_1_306().bcs_cddavailable(),
                   1);  //
  EXPECT_DOUBLE_EQ(chassis_detail.scu_bcs_1_306().bcs_brkpedact(),
                   0.8);                                                   //
  EXPECT_DOUBLE_EQ(chassis_detail.scu_bcs_1_306().bcs_intidx(), 0);  //
  EXPECT_DOUBLE_EQ(chassis_detail.scu_bcs_1_306().bcs_vdcfaultst(),
                   0);  //
  EXPECT_DOUBLE_EQ(chassis_detail.scu_bcs_1_306().bcs_vdcactivest(),
                   0);  //
  EXPECT_DOUBLE_EQ(chassis_detail.scu_bcs_1_306().bcs_absfaultst(),
                   0);  //
  EXPECT_DOUBLE_EQ(chassis_detail.scu_bcs_1_306().bcs_absactivest(),
                   0);                                                      //
  EXPECT_DOUBLE_EQ(chassis_detail.scu_bcs_1_306().bcs_faultst(), 0);  //
  EXPECT_DOUBLE_EQ(chassis_detail.scu_bcs_1_306().bcs_drvmode(), 0);  //
}

}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
