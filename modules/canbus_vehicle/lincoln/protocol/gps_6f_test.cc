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

#include "modules/canbus_vehicle/lincoln/protocol/gps_6f.h"

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace lincoln {

TEST(Gps6fTest, General) {
  uint8_t data[8] = {0x56, 0x52, 0x53, 0x54, 0xF1, 0xF2, 0xF3, 0xF4};
  int32_t length = 8;
  Lincoln cd;
  Gps6f gps;
  gps.Parse(data, length, &cd);

  EXPECT_TRUE(cd.has_basic());
  EXPECT_TRUE(cd.basic().has_altitude());
  EXPECT_TRUE(cd.basic().has_heading());
  EXPECT_TRUE(cd.basic().has_gps_speed());

  EXPECT_DOUBLE_EQ(cd.basic().altitude(), 5269.5);
  EXPECT_DOUBLE_EQ(cd.basic().heading(), 215.87);

  EXPECT_DOUBLE_EQ(cd.basic().hdop(), 3.6);
  EXPECT_DOUBLE_EQ(cd.basic().vdop(), 3.8);
  EXPECT_DOUBLE_EQ(cd.basic().quality(), FIX_INVALID);
  EXPECT_DOUBLE_EQ(cd.basic().gps_speed(), 107.73664);
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
