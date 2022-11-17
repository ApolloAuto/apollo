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

#include "modules/canbus/vehicle/lincoln/protocol/tirepressure_71.h"

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace lincoln {

TEST(Tirepressure71Test, General) {
  uint8_t data[8] = {0x67, 0x62, 0x63, 0x64, 0x51, 0x52, 0x53, 0x54};
  int32_t length = 8;

  Tirepressure71 tire_pressure;
  ChassisDetail cd;
  tire_pressure.Parse(data, length, &cd);

  EXPECT_EQ(cd.safety().front_left_tire_press(), 25191);
  EXPECT_EQ(cd.safety().front_right_tire_press(), 25699);
  EXPECT_EQ(cd.safety().rear_left_tire_press(), 21073);
  EXPECT_EQ(cd.safety().rear_right_tire_press(), 21587);
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
