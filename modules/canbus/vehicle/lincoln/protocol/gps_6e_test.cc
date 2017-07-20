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

#include "modules/canbus/vehicle/lincoln/protocol/gps_6e.h"

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace lincoln {

TEST(Gps6eTest, General) {
  uint8_t data[8] = {0x56, 0x52, 0x53, 0x54, 0xF1, 0xF2, 0xF3, 0xF4};
  int32_t length = 8;
  ChassisDetail cd;
  Gps6e gps;
  gps.Parse(data, length, &cd);

  EXPECT_EQ(cd.basic().year(), 86);
  EXPECT_EQ(cd.basic().month(), 2);
  EXPECT_EQ(cd.basic().day(), 19);
  EXPECT_EQ(cd.basic().hours(), 20);
  EXPECT_EQ(cd.basic().minutes(), 49);
  EXPECT_EQ(cd.basic().seconds(), 50);
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
