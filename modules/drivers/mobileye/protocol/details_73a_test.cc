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

#include "modules/drivers/mobileye/protocol/details_73a.h"

#include "gtest/gtest.h"

namespace apollo {
namespace drivers {
namespace mobileye {

class Details73aTest : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Details73aTest, Parse) {
  Details73a details_73a;
  int32_t length = 8;
  Mobileye mobileye;
  uint8_t bytes[8];

  bytes[0] = 0b00010001;
  bytes[1] = 0b10000100;
  bytes[2] = 0b00000011;
  bytes[3] = 0b00000000;
  bytes[4] = 0b00000000;
  bytes[5] = 0b00000000;
  bytes[6] = 0b00000000;
  bytes[7] = 0b00000000;

  details_73a.Parse(bytes, length, &mobileye);

  EXPECT_EQ(1, mobileye.details_73a_size());
  EXPECT_NEAR(8.5, mobileye.details_73a(0).obstacle_length(), 1e-6);
  EXPECT_NEAR(6.6, mobileye.details_73a(0).obstacle_width(), 1e-6);
  EXPECT_EQ(3, mobileye.details_73a(0).obstacle_age());
}

}  // namespace mobileye
}  // namespace drivers
}  // namespace apollo
