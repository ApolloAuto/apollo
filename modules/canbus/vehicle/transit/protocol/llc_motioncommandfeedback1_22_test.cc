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

#include "modules/canbus/vehicle/transit/protocol/llc_motioncommandfeedback1_22.h"

#include "gtest/gtest.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace transit {

using ::apollo::drivers::canbus::Byte;

class Motioncommandfeedback1_22_test : public ::testing::Test {
 public:
  virtual void SetUp() {}

 protected:
  Llcmotioncommandfeedback122 feedback_;
};

TEST_F(Motioncommandfeedback1_22_test, Steeringanglesetpoint) {
  const int32_t length = 16;
  const uint8_t bytes[8] = {0x8f, 0x9e, 0xad, 0xbc, 0xcb, 0xda, 0xe9, 0xf8};
  const double equivalent = 1201.85;
  EXPECT_DOUBLE_EQ(feedback_.llc_fbk_steeringanglesetpoint(bytes, length),
                    equivalent);
}

TEST_F(Motioncommandfeedback1_22_test, Throttlesetpoint) {
  const int32_t length = 10;
  const uint8_t bytes[8] = {0x8f, 0x9e, 0xad, 0xbc, 0xcb, 0xda, 0xe9, 0xf8};
  const double equivalent = 43.50;
  EXPECT_DOUBLE_EQ(feedback_.llc_fbk_throttlesetpoint(bytes, length),
                    equivalent);
}

TEST_F(Motioncommandfeedback1_22_test, Brakepercentsetpoint) {
  const int32_t length = 11;
  const uint8_t bytes[8] = {0x8f, 0x9e, 0xad, 0xbc, 0xcb, 0xda, 0xe9, 0xf8};
  const double equivalent = 93.3524;
  EXPECT_DOUBLE_EQ(feedback_.llc_fbk_brakepercentsetpoint(bytes, length),
                    equivalent);
}

TEST_F(Motioncommandfeedback1_22_test, Motioncommandfeedback1_count) {
  const int32_t length = 2;
  const uint8_t bytes[8] = {0x8f, 0x9e, 0xad, 0xbc, 0xcb, 0xda, 0xe9, 0xf8};
  const int equivalent = 3;
  EXPECT_EQ(feedback_.llc_motioncommandfeedback1_count(bytes, length),
            equivalent);
}

TEST_F(Motioncommandfeedback1_22_test, Motioncommandfeedback1_check) {
  const int32_t length = 8;
  const uint8_t bytes[8] = {0x8f, 0x9e, 0xad, 0xbc, 0xcb, 0xda, 0xe9, 0xf8};
  const int equivalent = 248;
  EXPECT_EQ(feedback_.llc_motioncommandfeedback1_check(bytes, length),
            equivalent);
}

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
