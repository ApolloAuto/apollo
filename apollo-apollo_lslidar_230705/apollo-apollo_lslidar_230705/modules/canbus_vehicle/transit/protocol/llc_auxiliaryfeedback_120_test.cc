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

#include "modules/canbus_vehicle/transit/protocol/llc_auxiliaryfeedback_120.h"
#include "gtest/gtest.h"

#include "modules/canbus_vehicle/transit/proto/transit.pb.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace transit {

using ::apollo::drivers::canbus::Byte;

class llc_auxiliaryfeedback_120Test : public ::testing ::Test {
 public:
  virtual void SetUp() {}

 protected:
  Llcauxiliaryfeedback120 Llcauxiliary_feedback120_;
};

TEST_F(llc_auxiliaryfeedback_120Test, General) {
  const uint8_t kData[4] = {0xFB, 0xFF, 0xFF, 0xFF};
  int32_t length = 1;
  int32_t turnsignal_len = 2;
  EXPECT_FALSE(Llcauxiliary_feedback120_.llc_fbk_inverter(kData, length));
  EXPECT_TRUE(Llcauxiliary_feedback120_.llc_fbk_pdu_ch8(kData, length));
  EXPECT_TRUE(Llcauxiliary_feedback120_.llc_fbk_pdu_ch7(kData, length));
  EXPECT_TRUE(Llcauxiliary_feedback120_.llc_fbk_pdu_ch6(kData, length));
  EXPECT_TRUE(Llcauxiliary_feedback120_.llc_fbk_pdu_ch5(kData, length));
  EXPECT_TRUE(Llcauxiliary_feedback120_.llc_fbk_pdu_ch4(kData, length));
  EXPECT_TRUE(Llcauxiliary_feedback120_.llc_fbk_pdu_ch3(kData, length));
  EXPECT_TRUE(Llcauxiliary_feedback120_.llc_fbk_pdu_ch2(kData, length));
  EXPECT_TRUE(Llcauxiliary_feedback120_.llc_fbk_pdu_ch1(kData, length));
  EXPECT_TRUE(Llcauxiliary_feedback120_.llc_fbk_hazardlights(kData, length));
  EXPECT_TRUE(Llcauxiliary_feedback120_.llc_fbk_ledgreenon(kData, length));
  EXPECT_TRUE(Llcauxiliary_feedback120_.llc_fbk_horn(kData, length));
  EXPECT_TRUE(Llcauxiliary_feedback120_.llc_fbk_buzzeron(kData, length));

  EXPECT_EQ(Llcauxiliary_feedback120_.llc_fbk_turnsignal(kData, turnsignal_len),
            3);

  EXPECT_TRUE(Llcauxiliary_feedback120_.llc_fbk_lowbeam(kData, length));
  EXPECT_TRUE(Llcauxiliary_feedback120_.llc_fbk_highbeam(kData, length));
  EXPECT_TRUE(Llcauxiliary_feedback120_.llc_fbk_ledredon(kData, length));
  EXPECT_TRUE(
      Llcauxiliary_feedback120_.llc_fbk_autonomybuttonpressed(kData, length));
}

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
