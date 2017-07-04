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

#include "modules/canbus/vehicle/lincoln/protocol/gps_6d.h"

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace lincoln {

TEST(Gps6dTest, General) {
  uint8_t data = 0x01;
  int32_t length = 8;
  ChassisDetail cd;
  Gps6d gps;
  gps.Parse(&data, length, &cd);

  EXPECT_FALSE(cd.basic().gps_valid());
  EXPECT_NEAR(cd.basic().latitude(), 6.83e-4, 1e-5);
  EXPECT_NEAR(cd.basic().longitude(), 8.53e-5, 1e-5);
}

}  // namespace lincoln
}  // namespace apollo
}  // namespace canbus
