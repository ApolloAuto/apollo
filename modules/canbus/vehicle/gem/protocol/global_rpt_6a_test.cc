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

#include "modules/canbus/vehicle/gem/protocol/global_rpt_6a.h"

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace gem {

class Globalrpt6aTest : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Globalrpt6aTest, reset) {
  Globalrpt6a globalrpt;
  int32_t length = 8;
  ChassisDetail chassis_detail;
  uint8_t bytes[8] = {0x01, 0x02, 0x03, 0x04, 0x11, 0x12, 0x13, 0x14};
  globalrpt.Parse(bytes, length, &chassis_detail);
  EXPECT_DOUBLE_EQ(chassis_detail.gem().global_rpt_6a().pacmod_status(), 1);
  EXPECT_DOUBLE_EQ(chassis_detail.gem().global_rpt_6a().override_status(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.gem().global_rpt_6a().veh_can_timeout(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.gem().global_rpt_6a().str_can_timeout(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.gem().global_rpt_6a().brk_can_timeout(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.gem().global_rpt_6a().usr_can_timeout(), 0);
  EXPECT_DOUBLE_EQ(chassis_detail.gem().global_rpt_6a().usr_can_read_errors(),
                   4884);
}

}  // namespace gem
}  // namespace canbus
}  // namespace apollo
