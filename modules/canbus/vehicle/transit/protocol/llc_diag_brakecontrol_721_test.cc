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

#include "modules/canbus/vehicle/transit/protocol/llc_diag_brakecontrol_721.h"
#include "gtest/gtest.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace transit {
using ::apollo::drivers::canbus::Byte;

class llc_diag_brakecontrol_721Test : public ::testing ::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(llc_diag_brakecontrol_721Test, part1) {
  Llcdiagbrakecontrol721 brakectrl_721_;
  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  double x = 0.0;

  brakectrl_721_.set_p_llc_dbg_brakepidcontribution_p(data, x);
  brakectrl_721_.set_p_llc_dbg_brakepidcontribution_d(data, x);
  brakectrl_721_.set_p_llc_dbg_brakepid_output(data, x);

  EXPECT_EQ(data[1], 0xF);
  EXPECT_EQ(data[2], 0xC0);
  EXPECT_EQ(data[4], 0x3);
  EXPECT_EQ(data[5], 0xF0);
  EXPECT_EQ(data[6], 0x3F);
  EXPECT_EQ(data[7], 0x0);
}

TEST_F(llc_diag_brakecontrol_721Test, part2) {
  Llcdiagbrakecontrol721 brakectrl_721_;
  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  double x = 0.0;
  int a = 0;

  brakectrl_721_.set_p_llc_dbg_brakepidcontribution_i(data, x);
  brakectrl_721_.set_p_llc_dbg_brakepid_error(data, a);
  brakectrl_721_.set_p_llc_dbg_brakefeedforward(data, x);

  EXPECT_EQ(data[0], 0x0);
  EXPECT_EQ(data[1], 0xF0);
  EXPECT_EQ(data[2], 0x3F);
  EXPECT_EQ(data[3], 0x0);
  EXPECT_EQ(data[4], 0xFC);
  EXPECT_EQ(data[5], 0xF);
  EXPECT_EQ(data[6], 0xC0);
}

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
