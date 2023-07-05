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

#include "modules/canbus_vehicle/transit/protocol/llc_motionfeedback1_20.h"

#include "gtest/gtest.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace transit {

using ::apollo::drivers::canbus::Byte;

class Motionfeedback1_20_test : public ::testing::Test {
 public:
  virtual void SetUp() {}

 protected:
  Llcmotionfeedback120 feedback_;
};

TEST_F(Motionfeedback1_20_test, General) {
  const uint8_t bytes[8] = {0x8f, 0x9e, 0xad, 0xbc, 0xcb, 0xda, 0xe9, 0xf8};
  const int32_t length_Gear = 3;
  const int32_t length_Parkingbrake = 1;
  const int32_t length_Throttleposition = 10;
  const int32_t length_Brakepercentrear = 11;
  const int32_t length_Brakepercentfront = 11;
  const int32_t length_Steeringcontrolmode = 2;
  const int32_t length_Motionfeedback1_counter = 2;
  const int32_t length_Motionfeedback1_checksum = 8;
  const int32_t length_Commandaligned = 1;
  const int32_t length_Estoppressed = 1;
  const int32_t length_Adcrequestautonomy = 1;
  const int32_t length_Allowautonomy = 1;
  const int32_t length_Longitudinalcontrolmode = 2;
  const int32_t length_State = 4;

  const int32_t equivalent_Gear = 2;
  const double equivalent_Throttleposition = 87.50;
  const double equivalent_Brakepercentrear = 20.85;
  const double equivalent_Brakepercentfront = 66.5532;
  const int32_t equivalent_Steeringcontrolmode = 2;
  const int32_t equivalent_Motionfeedback1_counter = 3;
  const int32_t equivalent_Motionfeedback1_checksum = 248;
  const int32_t equivalent_Longitudinalcontrolmode = 0;
  const int32_t equivalent_State = 15;

  EXPECT_EQ(
      feedback_.llc_fbk_gear(bytes, length_Gear),
      static_cast<Llc_motionfeedback1_20 ::Llc_fbk_gearType>(equivalent_Gear));
  EXPECT_TRUE(feedback_.llc_fbk_parkingbrake(bytes, length_Parkingbrake));
  EXPECT_DOUBLE_EQ(
      feedback_.llc_fbk_throttleposition(bytes, length_Throttleposition),
      equivalent_Throttleposition);
  EXPECT_DOUBLE_EQ(
      feedback_.llc_fbk_brakepercentrear(bytes, length_Brakepercentrear),
      equivalent_Brakepercentrear);
  EXPECT_DOUBLE_EQ(
      feedback_.llc_fbk_brakepercentfront(bytes, length_Brakepercentfront),
      equivalent_Brakepercentfront);
  EXPECT_EQ(
      feedback_.llc_fbk_steeringcontrolmode(bytes, length_Steeringcontrolmode),
      static_cast<Llc_motionfeedback1_20 ::Llc_fbk_steeringcontrolmodeType>(
          equivalent_Steeringcontrolmode));
  EXPECT_EQ(feedback_.llc_motionfeedback1_counter(
                bytes, length_Motionfeedback1_counter),
            equivalent_Motionfeedback1_counter);
  EXPECT_EQ(feedback_.llc_motionfeedback1_checksum(
                bytes, length_Motionfeedback1_checksum),
            equivalent_Motionfeedback1_checksum);
  EXPECT_TRUE(
      feedback_.llc_motionfeedback1_checksum(bytes, length_Commandaligned));
  EXPECT_TRUE(feedback_.llc_fbk_estoppressed(bytes, length_Estoppressed));
  EXPECT_TRUE(
      feedback_.llc_fbk_adcrequestautonomy(bytes, length_Adcrequestautonomy));
  EXPECT_FALSE(feedback_.llc_fbk_allowautonomy(bytes, length_Allowautonomy));
  EXPECT_EQ(
      feedback_.llc_fbk_longitudinalcontrolmode(bytes,
                                                length_Longitudinalcontrolmode),
      static_cast<Llc_motionfeedback1_20 ::Llc_fbk_longitudinalcontrolmodeType>(
          equivalent_Longitudinalcontrolmode));
  EXPECT_EQ(feedback_.llc_fbk_state(bytes, length_State),
            static_cast<Llc_motionfeedback1_20 ::Llc_fbk_stateType>(
                equivalent_State));
}

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
