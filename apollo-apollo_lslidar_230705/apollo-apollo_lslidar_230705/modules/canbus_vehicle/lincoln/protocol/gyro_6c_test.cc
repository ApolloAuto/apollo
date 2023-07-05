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

#include "modules/canbus_vehicle/lincoln/protocol/gyro_6c.h"

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace lincoln {

TEST(Gyro6cTest, General) {
  uint8_t data[8] = {0x67, 0x62, 0x63, 0x64, 0x51, 0x52, 0x53, 0x54};
  int32_t length = 8;
  Lincoln cd;
  Gyro6c gyro;
  gyro.Parse(data, length, &cd);

  EXPECT_TRUE(cd.vehicle_spd().is_yaw_rate_valid());
  EXPECT_DOUBLE_EQ(cd.vehicle_spd().yaw_rate(), 5.1398);
  EXPECT_DOUBLE_EQ(cd.vehicle_spd().roll_rate(), 5.0382);
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
