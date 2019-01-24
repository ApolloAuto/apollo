/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/vehicle/wey/protocol/vin_resp3_393.h"
#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace wey {

class Vinresp3393Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Vinresp3393Test, reset) {
  Vinresp3393 vin3;
  int32_t length = 8;
  ChassisDetail chassis_detail;
  uint8_t bytes[8] = {0x88, 0x44, 0x22, 0x11, 0x11, 0x12, 0x13, 0x14};

  vin3.Parse(bytes, length, &chassis_detail);
  EXPECT_DOUBLE_EQ(chassis_detail.wey().vin_resp3_393().vin16(), 136);
}

}  // namespace wey
}  // namespace canbus
}  // namespace apollo
