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

#include "modules/canbus/vehicle/transit/protocol/llc_auxiliaryfeedback_120.h"
#include "gtest/gtest.h"

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
  const std::uint8_t kBytes = 0xFF;
  std::int32_t length = 1;
  EXPECT_TRUE(Llcauxiliary_feedback120_.llc_fbk_inverter(&kBytes, length));
  EXPECT_FALSE(Llcauxiliary_feedback120_.llc_fbk_pdu_ch8(&kBytes, length));
  EXPECT_FALSE(Llcauxiliary_feedback120_.llc_fbk_pdu_ch7(&kBytes, length));
  EXPECT_FALSE(Llcauxiliary_feedback120_.llc_fbk_pdu_ch6(&kBytes, length));
  EXPECT_FALSE(Llcauxiliary_feedback120_.llc_fbk_pdu_ch5(&kBytes, length));
  EXPECT_FALSE(Llcauxiliary_feedback120_.llc_fbk_pdu_ch4(&kBytes, length));
  EXPECT_FALSE(Llcauxiliary_feedback120_.llc_fbk_pdu_ch3(&kBytes, length));
  EXPECT_FALSE(Llcauxiliary_feedback120_.llc_fbk_pdu_ch2(&kBytes, length));
  EXPECT_TRUE(Llcauxiliary_feedback120_.llc_fbk_pdu_ch1(&kBytes, length));
  EXPECT_FALSE(Llcauxiliary_feedback120_.llc_fbk_hazardlights(&kBytes, length));
  EXPECT_FALSE(Llcauxiliary_feedback120_.llc_fbk_ledgreenon(&kBytes, length));
  EXPECT_TRUE(Llcauxiliary_feedback120_.llc_fbk_horn(&kBytes, length));
  EXPECT_FALSE(Llcauxiliary_feedback120_.llc_fbk_buzzeron(&kBytes, length));
  EXPECT_FALSE(Llcauxiliary_feedback120_.llc_fbk_turnsignal(&kBytes, length));
  EXPECT_FALSE(Llcauxiliary_feedback120_.llc_fbk_lowbeam(&kBytes, length));
  EXPECT_FALSE(Llcauxiliary_feedback120_.llc_fbk_highbeam(&kBytes, length));
  EXPECT_FALSE(Llcauxiliary_feedback120_.llc_fbk_ledredon(&kBytes, length));
  EXPECT_FALSE(
      Llcauxiliary_feedback120_.llc_fbk_autonomybuttonpressed(&kBytes, length));
}

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
