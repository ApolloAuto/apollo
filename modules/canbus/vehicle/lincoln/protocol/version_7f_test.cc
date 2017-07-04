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

#include "modules/canbus/vehicle/lincoln/protocol/version_7f.h"

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace lincoln {

TEST(Version7fTest, General) {
  Version7f version;
  uint8_t data = 0x03;
  int32_t length = 8;
  ChassisDetail cd;
  version.Parse(&data, length, &cd);
  
  EXPECT_EQ(cd.eps().major_version(), 0);
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
