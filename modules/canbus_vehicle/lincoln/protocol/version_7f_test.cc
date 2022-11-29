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

#include "modules/canbus_vehicle/lincoln/protocol/version_7f.h"

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace lincoln {

TEST(Version7fTest, General) {
  int32_t length = 8;
  Lincoln cd;
  Version7f version;

  uint8_t data[8] = {0x01, 0x62, 0x63, 0x64, 0x51, 0x52, 0x53, 0x54};
  version.Parse(data, length, &cd);
  EXPECT_TRUE(cd.has_brake());
  EXPECT_FALSE(cd.has_gas());
  EXPECT_FALSE(cd.has_eps());

  EXPECT_EQ(cd.brake().major_version(), 25699);
  EXPECT_EQ(cd.brake().minor_version(), 21073);
  EXPECT_EQ(cd.brake().build_number(), 21587);

  data[0] = 0x02;
  cd.Clear();
  version.Parse(data, length, &cd);
  EXPECT_FALSE(cd.has_brake());
  EXPECT_TRUE(cd.has_gas());
  EXPECT_FALSE(cd.has_eps());
  EXPECT_EQ(cd.gas().major_version(), 25699);
  EXPECT_EQ(cd.gas().minor_version(), 21073);
  EXPECT_EQ(cd.gas().build_number(), 21587);

  data[0] = 0x03;
  cd.Clear();
  version.Parse(data, length, &cd);
  EXPECT_FALSE(cd.has_brake());
  EXPECT_FALSE(cd.has_gas());
  EXPECT_TRUE(cd.has_eps());
  EXPECT_EQ(cd.eps().major_version(), 25699);
  EXPECT_EQ(cd.eps().minor_version(), 21073);
  EXPECT_EQ(cd.eps().build_number(), 21587);
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
