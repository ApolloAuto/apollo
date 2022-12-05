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

#include "modules/canbus_vehicle/gem/protocol/wiper_rpt_91.h"

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace gem {

class Wiperrpt91Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Wiperrpt91Test, reset) {
  Wiperrpt91 wiper;
  int32_t length = 8;
  Gem chassis_detail;
  uint8_t bytes[3] = {0x01, 0x02, 0x03};
  wiper.Parse(bytes, length, &chassis_detail);
  EXPECT_DOUBLE_EQ(chassis_detail.wiper_rpt_91().manual_input(), 1);
  EXPECT_DOUBLE_EQ(chassis_detail.wiper_rpt_91().commanded_value(), 2);
  EXPECT_DOUBLE_EQ(chassis_detail.wiper_rpt_91().output_value(), 3);
}

}  // namespace gem
}  // namespace canbus
}  // namespace apollo
