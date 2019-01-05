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

class Motioncommandfeedback1_22_Test : public ::testing::Test {
 public:
 protected:
Llcmotioncommandfeedback122 feedback_;
};

uint8_t tmpByte = 63;
uint8_t* byte = &tmpByte;
ChassisDetail tmpChassis;
ChassisDetail* chassis = &tmpChassis;
int32_t len = 16;
double eq = -1638.4;

TEST_F(Motioncommandfeedback1_22_Test, SteeringTest) {
EXPECT_EQ(feedback_.llc_fbk_steeringanglesetpoint(byte, len), eq);
}


}  // namespace transit
}  // namespace canbus
}  // namespace apollo
