/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/vehicle/lincoln/protocol/surround_73.h"

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace lincoln {

TEST(Surround73Test, General) {
  uint8_t data[8] = {0x67, 0x62, 0x63, 0x64, 0x51, 0x00, 0x53, 0x54};
  int32_t length = 8;
  ChassisDetail cd;
  Surround73 surround;
  surround.Parse(data, length, &cd);

  EXPECT_TRUE(cd.surround().cross_traffic_alert_left());
  EXPECT_TRUE(cd.surround().cross_traffic_alert_left_enabled());
  EXPECT_TRUE(cd.surround().blind_spot_left_alert());
  EXPECT_FALSE(cd.surround().blind_spot_left_alert_enabled());

  EXPECT_FALSE(cd.surround().cross_traffic_alert_right());
  EXPECT_FALSE(cd.surround().cross_traffic_alert_right_enabled());
  EXPECT_TRUE(cd.surround().blind_spot_right_alert());
  EXPECT_FALSE(cd.surround().blind_spot_right_alert_enabled());

  EXPECT_DOUBLE_EQ(cd.surround().sonar00(), 0.44499999999999995);
  EXPECT_DOUBLE_EQ(cd.surround().sonar01(), 1.0249999999999999);
  EXPECT_DOUBLE_EQ(cd.surround().sonar02(), 0.58999999999999997);
  EXPECT_DOUBLE_EQ(cd.surround().sonar03(), 1.0249999999999999);
  EXPECT_DOUBLE_EQ(cd.surround().sonar04(), 0.73499999999999988);
  EXPECT_DOUBLE_EQ(cd.surround().sonar05(), 1.0249999999999999);
  EXPECT_DOUBLE_EQ(cd.surround().sonar06(), 0.29999999999999999);
  EXPECT_DOUBLE_EQ(cd.surround().sonar07(), 0.87999999999999989);
  EXPECT_DOUBLE_EQ(cd.surround().sonar08(), 100);
  EXPECT_DOUBLE_EQ(cd.surround().sonar09(), 100);
  EXPECT_DOUBLE_EQ(cd.surround().sonar10(), 0.58999999999999997);
  EXPECT_DOUBLE_EQ(cd.surround().sonar11(), 0.87999999999999989);

  EXPECT_FALSE(cd.surround().sonar_enabled());
  EXPECT_FALSE(cd.surround().sonar_fault());
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
