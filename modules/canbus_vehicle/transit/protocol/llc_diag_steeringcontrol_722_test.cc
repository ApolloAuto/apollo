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

#include "modules/canbus_vehicle/transit/protocol/llc_diag_steeringcontrol_722.h"
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
  uint8_t data[7] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  EXPECT_EQ(steeringctrl_722_.GetPeriod(), 10 * 1000);

  double pos = 3;
  int torque = 0xF;

  steeringctrl_722_.set_llc_dbg_steeringsensorposition(pos);
  steeringctrl_722_.set_llc_dbg_steeringrackinputtorque(torque);
  steeringctrl_722_.set_llc_dbg_steeringmotorposition(pos);

  steeringctrl_722_.UpdateData(data);

  EXPECT_EQ(data[0], 0xE0);
  EXPECT_EQ(data[1], 0x93);
  EXPECT_EQ(data[2], 0x4);
  EXPECT_EQ(data[3], 0xF);
  EXPECT_EQ(data[4], 0x0);
  EXPECT_EQ(data[5], 0x98);
  EXPECT_EQ(data[6], 0x3A);
}

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
