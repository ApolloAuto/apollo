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

#include "modules/canbus_vehicle/transit/protocol/llc_vehiclelimits_24.h"

#include "gtest/gtest.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace transit {

using ::apollo::drivers::canbus::Byte;

class Vehiclelimits_24_test : public ::testing::Test {
 public:
  virtual void SetUp() {}

 protected:
  Llcvehiclelimits24 vehiclelimits_;
};

TEST_F(Vehiclelimits_24_test, General) {
  const uint8_t bytes[8] = {0x8f, 0x9e, 0xad, 0xbc, 0xcb, 0xda, 0xe9, 0xf8};
  const int32_t length_maxsteeringangle = 12;
  const int32_t length_maxbrakepercent = 12;
  const int equivalent_maxsteeringangle = 0xad9;
  const int equivalent_maxbrakepercent = 0xe8f;
  EXPECT_EQ(
      vehiclelimits_.llc_fbk_maxsteeringangle(bytes, length_maxsteeringangle),
      equivalent_maxsteeringangle);
  EXPECT_EQ(
      vehiclelimits_.llc_fbk_maxbrakepercent(bytes, length_maxbrakepercent),
      equivalent_maxbrakepercent);
}

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
