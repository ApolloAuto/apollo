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

#include "modules/canbus/vehicle/ge3/protocol/scu_3_303.h"
#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace ge3 {

class Scu3303Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Scu3303Test, reset) {
  Scu3303 scu3303;
  int32_t length = 8;
  ChassisDetail chassis_detail;
  uint8_t bytes[8] = {0x41, 0x42, 0x43, 0x61, 0x62, 0x30, 0x31, 0x32};

  scu3303.Parse(bytes, length, &chassis_detail);
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_3_303().vin08(), 'A');  // 65
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_3_303().vin09(), 'B');  // 66
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_3_303().vin10(), 67);   // 'C'
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_3_303().vin11(), 97);   // 'a'
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_3_303().vin12(), 'b');  // 98
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_3_303().vin13(), 48);   // '0'
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_3_303().vin14(), '1');  // 49
  EXPECT_DOUBLE_EQ(chassis_detail.ge3().scu_3_303().vin15(), '2');  // 50
}

}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
