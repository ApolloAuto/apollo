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

#include "modules/drivers/mobileye/protocol/details_73b.h"

#include "gtest/gtest.h"

namespace apollo {
namespace drivers {
namespace mobileye {

class Details73bTest : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Details73bTest, Parse) {
  Details73b details_73b;
  int32_t length = 8;
  Mobileye mobileye;
  uint8_t bytes[8];

  bytes[0] = 0b00000000;
  bytes[1] = 0b00000000;
  bytes[2] = 0b00000000;
  bytes[3] = 0b00000000;
  bytes[4] = 0b00000000;
  bytes[5] = 0b00010000;
  bytes[6] = 0b00110000;
  bytes[7] = 0b00000111;

  details_73b.Parse(bytes, length, &mobileye);

  EXPECT_EQ(1, mobileye.details_73b_size());
  EXPECT_EQ(true, mobileye.details_73b(0).obstacle_replaced());
  EXPECT_NEAR(18.4, mobileye.details_73b(0).obstacle_angle(), 1e-6);
}

}  // namespace mobileye
}  // namespace drivers
}  // namespace apollo
