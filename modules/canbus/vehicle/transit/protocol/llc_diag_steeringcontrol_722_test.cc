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

#include "modules/canbus/vehicle/transit/protocol/llc_diag_steeringcontrol_722.h"
#include "gtest/gtest.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace transit {
using ::apollo::drivers::canbus::Byte;

class llc_diag_steeringcontrol_722Test : public ::testing ::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(llc_diag_steeringcontrol_722Test, General) {
  Llcdiagsteeringcontrol722 steeringctrl_722_;
  uint8_t data[8] = {0x67, 0x62, 0x63, 0x64, 0x51, 0x52, 0x53};
  EXPECT_EQ(steeringctrl_722_.GetPeriod(), 10 * 1000);
  steeringctrl_722_.UpdateData(data);
  EXPECT_EQ(data[0], 0b00000000);
  EXPECT_EQ(data[1], 0b00000000);
  EXPECT_EQ(data[2], 0b00000000);
  EXPECT_EQ(data[3], 0b00000000);
  EXPECT_EQ(data[4], 0b00000000);
  EXPECT_EQ(data[5], 0b00000000);
  EXPECT_EQ(data[6], 0b00000000);
}

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
