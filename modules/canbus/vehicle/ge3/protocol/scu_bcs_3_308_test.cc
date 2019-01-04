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

#include "modules/canbus/vehicle/ge3/protocol/scu_bcs_3_308.h"
#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace ge3 {

class Scubcs3308Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Scubcs3308Test, reset) {
  Scubcs3308 scubcs3308;
  int32_t length = 8;
  ChassisDetail chassis_detail;
  uint8_t bytes[8] = {0x01, 0x02, 0x03, 0x04, 0x11, 0x12, 0x13, 0x14};

  scubcs3308.Parse(bytes, length, &chassis_detail);
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_bcs_3_308().bcs_rrwheelspdvd(),
                   0);  //
  EXPECT_DOUBLE_EQ(
      chassis_detail.ge3().scu_bcs_3_308().bcs_rrwheeldirectionvd(), 1);  //
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_bcs_3_308().bcs_rlwheelspdvd(),
                   1);  //
  EXPECT_DOUBLE_EQ(
      chassis_detail.ge3().scu_bcs_3_308().bcs_rlwheeldirectionvd(), 0);  //
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_bcs_3_308().bcs_frwheelspdvd(),
                   0);  //
  EXPECT_DOUBLE_EQ(
      chassis_detail.ge3().scu_bcs_3_308().bcs_frwheeldirectionvd(), 1);  //
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_bcs_3_308().bcs_flwheelspdvd(),
                   1);  //
  EXPECT_DOUBLE_EQ(
      chassis_detail.ge3().scu_bcs_3_308().bcs_flwheeldirectionvd(), 0);  //
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_bcs_3_308().bcs_rrwheelspd(),
                   34.3125);  //
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_bcs_3_308().bcs_rrwheeldirection(),
                   0);  //
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_bcs_3_308().bcs_rlwheelspd(),
                   30.7125);  //
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_bcs_3_308().bcs_rlwheeldirection(),
                   0);  //
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_bcs_3_308().bcs_frwheelspd(),
                   5.4);  //
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_bcs_3_308().bcs_frwheeldirection(),
                   0);  //
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_bcs_3_308().bcs_flwheelspd(),
                   1.8);  //
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_bcs_3_308().bcs_flwheeldirection(),
                   0);  //
}

}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
