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

#include "modules/canbus/vehicle/lincoln/protocol/brake_60.h"

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace lincoln {

class Brake60Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Brake60Test, simple) {
  Brake60 brake;
  EXPECT_EQ(brake.GetPeriod(), 20 * 1000);
  uint8_t data = 0x64;
  brake.UpdateData(&data);
}

}  // namespace lincoln
}  // namespace apollo
}  // namespace canbus
