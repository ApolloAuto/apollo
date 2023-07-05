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

#include "modules/canbus_vehicle/lincoln/protocol/throttle_63.h"

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace lincoln {

TEST(Throttle63Test, General) {
  uint8_t data[8] = {0x67, 0x62, 0x63, 0x64, 0x51, 0x52, 0x53, 0x54};
  int32_t length = 8;
  Lincoln cd;
  Throttle63 throttle;
  throttle.Parse(data, length, &cd);

  EXPECT_DOUBLE_EQ(cd.gas().throttle_input(), 38.439002059967905);
  EXPECT_DOUBLE_EQ(cd.gas().throttle_cmd(), 39.21416037232008);
  EXPECT_DOUBLE_EQ(cd.gas().throttle_output(), 32.155336842908326);
  EXPECT_EQ(cd.gas().watchdog_source(), 5);
  EXPECT_FALSE(cd.gas().throttle_enabled());
  EXPECT_FALSE(cd.gas().driver_override());
  EXPECT_TRUE(cd.gas().driver_activity());
  EXPECT_FALSE(cd.gas().watchdog_fault());
  EXPECT_TRUE(cd.gas().channel_1_fault());
  EXPECT_FALSE(cd.gas().channel_2_fault());
  EXPECT_FALSE(cd.gas().connector_fault());

  EXPECT_TRUE(cd.check_response().is_vcu_online());
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
