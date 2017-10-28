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

#include "modules/drivers/mobileye/protocol/details_738.h"

#include "gtest/gtest.h"

namespace apollo {
namespace drivers {
namespace mobileye {

class Details738Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Details738Test, Parse) {
  Details738 details_738;
  int32_t length = 8;
  Mobileye mobileye;
  uint8_t bytes[8];

  bytes[0] = 0b00000010;
  bytes[1] = 0b00000101;
  bytes[2] = 0b00000101;
  bytes[3] = 0b11111111;
  bytes[4] = 0b00000001;
  bytes[5] = 0b00000011;

  details_738.Parse(bytes, length, &mobileye);
  // TODO(lizh): add more checks
  EXPECT_EQ(2, mobileye.details_738().num_obstacles());
  EXPECT_EQ(15, mobileye.details_738().go());
}

}  // namespace mobileye
}  // namespace drivers
}  // namespace apollo
