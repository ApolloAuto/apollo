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

#include "modules/canbus/vehicle/transit/protocol/llc_motionfeedback2_21.h"
#include "gtest/gtest.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace transit {
using ::apollo::drivers::canbus::Byte;

class llc_motionfeedback2_21Test : public ::testing ::Test {
 public:
 protected:
  virtual void SetUp() {}
  Llcmotionfeedback221 Llcmotionfeedback2_21;
};

TEST_F(llc_motionfeedback2_21Test, motion_fdk) {
  const std::uint8_t bytes = 0x00;
  std::int32_t length = 1;
  EXPECT_EQ(Llcmotionfeedback2_21.llc_fbk_vehiclespeed(&bytes, length), 0);
  EXPECT_EQ(Llcmotionfeedback2_21.llc_motionfeedback2_counter(&bytes, length),
            0);
  EXPECT_EQ(Llcmotionfeedback2_21.llc_motionfeedback2_checksum(&bytes, length),
            0);
  EXPECT_EQ(Llcmotionfeedback2_21.llc_fbk_steeringrate(&bytes, length), 0);
  EXPECT_EQ(Llcmotionfeedback2_21.llc_fbk_steeringangle(&bytes, length), 12.8);
}

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
