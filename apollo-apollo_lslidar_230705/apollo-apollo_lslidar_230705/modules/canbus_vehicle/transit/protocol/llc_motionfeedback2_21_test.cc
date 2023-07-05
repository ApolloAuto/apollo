/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus_vehicle/transit/protocol/llc_motionfeedback2_21.h"
#include "gtest/gtest.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace transit {
using ::apollo::drivers::canbus::Byte;

class llc_motionfeedback2_21Test : public ::testing ::Test {
 public:
  virtual void SetUp() {}

 protected:
  Llcmotionfeedback221 motionfdk2_21_;
};

TEST_F(llc_motionfeedback2_21Test, motion_fdk) {
  const uint8_t kData[8] = {0x9A, 0xFC, 0x56, 0xF7, 0x12, 0x34, 0xFF, 0xFF};
  int32_t speed_len = 8;
  int32_t counter_len = 2;
  int32_t checksum_len = 8;
  int32_t steeringrate_len = 8;
  int32_t steeringangle_len = 8;

  EXPECT_DOUBLE_EQ(motionfdk2_21_.llc_fbk_vehiclespeed(kData, speed_len),
                   133.3);
  EXPECT_EQ(motionfdk2_21_.llc_motionfeedback2_counter(kData, counter_len), 3);
  EXPECT_EQ(motionfdk2_21_.llc_motionfeedback2_checksum(kData, checksum_len),
            0xFF);
  EXPECT_DOUBLE_EQ(motionfdk2_21_.llc_fbk_steeringrate(kData, steeringrate_len),
                   -110.9);
  EXPECT_DOUBLE_EQ(
      motionfdk2_21_.llc_fbk_steeringangle(kData, steeringangle_len), -43.5);
}

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
