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

#include "modules/canbus_vehicle/lincoln/protocol/throttleinfo_75.h"

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace lincoln {

TEST(Throttleinfo75Test, General) {
  uint8_t data[8] = {0x67, 0x62, 0x63, 0x64, 0x51, 0x52, 0x53, 0x54};
  int32_t length = 8;
  Lincoln cd;
  Throttleinfo75 throttle_info;
  throttle_info.Parse(data, length, &cd);

  EXPECT_TRUE(cd.has_ems());
  EXPECT_TRUE(cd.ems().has_engine_rpm());
  EXPECT_TRUE(cd.has_gas());
  EXPECT_TRUE(cd.gas().has_accelerator_pedal());
  EXPECT_TRUE(cd.gas().has_accelerator_pedal_rate());

  EXPECT_DOUBLE_EQ(cd.ems().engine_rpm(), 6297.75);
  EXPECT_DOUBLE_EQ(cd.gas().accelerator_pedal(), 9.9);
  EXPECT_DOUBLE_EQ(cd.gas().accelerator_pedal_rate(), -7);
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
