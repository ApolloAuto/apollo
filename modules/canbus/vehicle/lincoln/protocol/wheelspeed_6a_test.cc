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

#include "modules/canbus/vehicle/lincoln/protocol/wheelspeed_6a.h"

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace lincoln {

TEST(Wheelspeed6aTest, General) {
  Wheelspeed6a wheelspeed;
  uint8_t data = 0x64U;
  int32_t length = 8;
  ChassisDetail cd;
  struct timeval timestamp;
  wheelspeed.Parse(&data, length, timestamp, &cd);

  EXPECT_TRUE(cd.vehicle_spd().is_wheel_spd_fl_valid());
  EXPECT_TRUE(cd.vehicle_spd().is_wheel_spd_fr_valid());
  EXPECT_TRUE(cd.vehicle_spd().is_wheel_spd_rl_valid());
  EXPECT_TRUE(cd.vehicle_spd().is_wheel_spd_rr_valid());
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
