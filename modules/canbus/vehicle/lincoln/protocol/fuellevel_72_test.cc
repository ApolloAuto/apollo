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

#include "modules/canbus/vehicle/lincoln/protocol/fuellevel_72.h"

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace lincoln {

class Accel6bTest : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Accel6bTest, Parse) {
  Fuellevel72 fuel;
  uint8_t data = 0x12U;
  int32_t length = 8;
  ChassisDetail chassis_detail;
  fuel.Parse(&data, length, &chassis_detail);
  EXPECT_DOUBLE_EQ(chassis_detail.battery().fuel_level(), 224.565936);
}

}  // namespace lincoln
}  // namespace apollo
}  // namespace canbus
