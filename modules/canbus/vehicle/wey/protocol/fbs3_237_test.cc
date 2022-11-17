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

#include "modules/canbus/vehicle/wey/protocol/fbs3_237.h"
#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace wey {

class Fbs3237Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Fbs3237Test, reset) {
  Fbs3237 fbs3;
  int32_t length = 8;
  ChassisDetail chassis_detail;
  uint8_t bytes[8] = {0x88, 0x44, 0x22, 0x11, 0x11, 0x12, 0xFE, 0x14};

  fbs3.Parse(bytes, length, &chassis_detail);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().fbs3_237().engspd(), 4360.5);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().fbs3_237().accpedalpos(), 13.3858);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().fbs3_237().epbswtichposition(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().fbs3_237().currentgear(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().fbs3_237().eps_streeingmode(), 1);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().fbs3_237().epsdrvinputtrqvalue(),
                   -19.5508);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().fbs3_237().epsconsumedcurrvalue(), 127);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().fbs3_237().epscurrmod(), 2);
}

}  // namespace wey
}  // namespace canbus
}  // namespace apollo
