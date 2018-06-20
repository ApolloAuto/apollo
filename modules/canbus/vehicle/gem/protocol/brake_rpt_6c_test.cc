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

#include "modules/canbus/vehicle/gem/protocol/brake_rpt_6c.h"

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace gem {

TEST(Brake61Test, General) {
  int32_t length = 8;
  ChassisDetail chassis_detail;
  uint8_t bytes[8] = {0x01, 0x02, 0x03, 0x04, 0x11, 0x12, 0x13, 0x14};

  Brakerpt6c brake;
  brake.Parse(bytes, length, &chassis_detail);

  auto &brakerpt = chassis_detail.gem().brake_rpt_6c();
  EXPECT_DOUBLE_EQ(brakerpt.manual_input(), 0.258);
  EXPECT_DOUBLE_EQ(brakerpt.commanded_value(), 0.772);
  EXPECT_DOUBLE_EQ(brakerpt.output_value(), 4.37);
  EXPECT_EQ(brakerpt.brake_on_off(), Brake_rpt_6c::BRAKE_ON_OFF_ON);
}

}  // namespace gem
}  // namespace canbus
}  // namespace apollo
