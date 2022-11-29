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

#include "modules/canbus_vehicle/lincoln/protocol/gear_67.h"

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace lincoln {

TEST(Gear67Test, General) {
  uint8_t data[8] = {0x56, 0x52, 0x53, 0x54, 0xF1, 0xF2, 0xF3, 0xF4};
  int32_t length = 8;
  Lincoln cd;
  Gear67 gear;
  int32_t state = gear.gear_state(data, 8);
  EXPECT_EQ(state, (data[0] & 0b00000111));
  EXPECT_EQ((data[0] >> 3 & 0b00000001), gear.is_driver_override(data, 8));
  gear.Parse(data, length, &cd);

  EXPECT_TRUE(cd.gear().is_shift_position_valid());
  EXPECT_EQ(cd.gear().gear_state(), Chassis::GEAR_INVALID);
  EXPECT_FALSE(cd.gear().driver_override());
  EXPECT_EQ(cd.gear().gear_cmd(), Chassis::GEAR_LOW);
  EXPECT_FALSE(cd.gear().canbus_fault());
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
