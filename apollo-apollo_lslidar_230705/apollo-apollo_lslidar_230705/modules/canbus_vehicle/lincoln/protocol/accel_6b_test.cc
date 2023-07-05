/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus_vehicle/lincoln/protocol/accel_6b.h"

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace lincoln {

class Accel6bTest : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Accel6bTest, Parse) {
  Accel6b acc;
  int32_t length = 8;
  Lincoln chassis_detail;
  uint8_t bytes[8] = {0, 0};

  bytes[0] = 0b11111100;
  bytes[1] = 0b11111110;
  bytes[2] = 0b11111110;
  bytes[3] = 0b11111110;
  bytes[4] = 0b11111110;
  bytes[5] = 0b11111110;
  acc.Parse(bytes, length, &chassis_detail);
  EXPECT_DOUBLE_EQ(chassis_detail.vehicle_spd().lat_acc(), -2.6);
  EXPECT_DOUBLE_EQ(chassis_detail.vehicle_spd().long_acc(), -2.58);
  EXPECT_DOUBLE_EQ(chassis_detail.vehicle_spd().vert_acc(), -2.58);
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
