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

#include "modules/canbus_vehicle/transit/protocol/llc_motioncommandfeedback1_22.h"

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

TEST_F(Motioncommandfeedback1_22_test, General) {
  const uint8_t bytes[8] = {0x8f, 0x9e, 0xad, 0xbc, 0xcb, 0xda, 0xe9, 0xf8};
  const int32_t length_Steeringanglesetpoint = 16;
  const int32_t length_Throttlesetpoint = 10;
  const int32_t length_Brakepercentsetpoint = 11;
  const int32_t length_Motioncommandfeedback1_count = 2;
  const int32_t length_Motioncommandfeedback1_check = 8;
  const double equivalent_Steeringanglesetpoint = 1201.85;
  const double equivalent_Throttlesetpoint = 43.50;
  const double equivalent_Brakepercentsetpoint = 93.3524;
  const int equivalent_Motioncommandfeedback1_count = 3;
  const int equivalent_Motioncommandfeedback1_check = 248;
  EXPECT_DOUBLE_EQ(feedback_.llc_fbk_steeringanglesetpoint(
                       bytes, length_Steeringanglesetpoint),
                   equivalent_Steeringanglesetpoint);
  EXPECT_DOUBLE_EQ(
      feedback_.llc_fbk_throttlesetpoint(bytes, length_Throttlesetpoint),
      equivalent_Throttlesetpoint);
  EXPECT_DOUBLE_EQ(feedback_.llc_fbk_brakepercentsetpoint(
                       bytes, length_Brakepercentsetpoint),
                   equivalent_Brakepercentsetpoint);
  EXPECT_EQ(feedback_.llc_motioncommandfeedback1_count(
                bytes, length_Motioncommandfeedback1_count),
            equivalent_Motioncommandfeedback1_count);
  EXPECT_EQ(feedback_.llc_motioncommandfeedback1_check(
                bytes, length_Motioncommandfeedback1_check),
            equivalent_Motioncommandfeedback1_check);
}

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
