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

#include "modules/canbus_vehicle/lincoln/protocol/steering_65.h"

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace lincoln {

TEST(Steering65Test, General) {
  uint8_t data[8] = {0x67, 0x62, 0x63, 0x64, 0x51, 0x52, 0x53, 0x54};
  int32_t length = 8;
  Lincoln cd;
  Steering65 steering;
  steering.Parse(data, length, &cd);

  EXPECT_FALSE(cd.eps().steering_enabled());

  EXPECT_TRUE(cd.vehicle_spd().is_vehicle_spd_valid());
  EXPECT_DOUBLE_EQ(cd.vehicle_spd().vehicle_spd(), 58.536111111111111);

  EXPECT_TRUE(cd.eps().is_steering_angle_valid());
  EXPECT_DOUBLE_EQ(cd.eps().steering_angle(), 2519.1);
  EXPECT_DOUBLE_EQ(cd.eps().steering_angle_cmd(), 2569.9);
  EXPECT_DOUBLE_EQ(cd.eps().vehicle_speed(), 58.536111111111111);
  EXPECT_DOUBLE_EQ(cd.eps().epas_torque(), 5.1875);
  EXPECT_FALSE(cd.eps().steering_enabled());
  EXPECT_FALSE(cd.eps().driver_override());
  EXPECT_TRUE(cd.eps().driver_activity());
  EXPECT_FALSE(cd.eps().watchdog_fault());
  EXPECT_TRUE(cd.eps().channel_1_fault());
  EXPECT_FALSE(cd.eps().channel_2_fault());
  EXPECT_TRUE(cd.eps().calibration_fault());
  EXPECT_FALSE(cd.eps().connector_fault());

  EXPECT_TRUE(cd.check_response().is_eps_online());
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
