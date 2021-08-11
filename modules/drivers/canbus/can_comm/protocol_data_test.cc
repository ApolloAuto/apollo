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

#include "modules/drivers/canbus/can_comm/protocol_data.h"

#include "gtest/gtest.h"

#include "modules/canbus/proto/chassis_detail.pb.h"

namespace apollo {
namespace drivers {
namespace canbus {

using ::apollo::canbus::ChassisDetail;

TEST(ProtocolDataTest, CheckSum) {
  const uint8_t INPUT[] = {0x00, 0x12, 0x00, 0x13, 0x00, 0xF3, 0x00, 0x00};
  const uint8_t result =
      ProtocolData<apollo::canbus::ChassisDetail>::CalculateCheckSum(INPUT, 8);
  EXPECT_EQ(0xE7, result);
}

TEST(ProtocolDataTest, BoundedValue) {
  const double_t input = 5.0;
  const double_t min_bound = 0.0;
  const double_t max_bound = M_PI;
  const double_t result =
      ProtocolData<apollo::canbus::ChassisDetail>::BoundedValue(
          min_bound, max_bound, input);
  EXPECT_EQ(M_PI, result);
}

}  // namespace canbus
}  // namespace drivers
}  // namespace apollo
