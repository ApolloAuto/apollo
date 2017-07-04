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

#include "modules/canbus/vehicle/lincoln/protocol/gear_66.h"

#include "gtest/gtest.h"

#include <iostream>

namespace apollo {
namespace canbus {
namespace lincoln {

class Accel6bTest : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Accel6bTest, Parse) {
  Gear66 gear;
  uint8_t data = 0x12U;
  EXPECT_EQ(gear.GetPeriod(), 20 * 1000);
  gear.UpdateData(&data);
}

}  // namespace lincoln
}  // namespace apollo
}  // namespace canbus
