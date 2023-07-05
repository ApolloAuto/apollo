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

#include "modules/canbus_vehicle/transit/protocol/adc_motioncontrollimits1_12.h"

#include "gtest/gtest.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace transit {

using ::apollo::drivers::canbus::Byte;

class Motioncontrollimits1_12_test : public ::testing::Test {
 public:
  virtual void SetUp() {}

 protected:
  Adcmotioncontrollimits112 controllimits_;
};

TEST_F(Motioncontrollimits1_12_test, General) {
  uint8_t data[8] = {0x8f, 0x9e, 0xad, 0xbc, 0xcb, 0xda, 0xe9, 0xf8};
  const double adc_cmd_throttlecommandlimit = 64.00;
  const double adc_cmd_steeringrate = 3071.15;
  const double adc_cmd_steerwheelanglelimit = 640.0;
  const uint8_t equivalent_throttlecommandlimit = 0x80;
  const uint8_t equivalent_steeringrate1 = 0xef;
  const uint8_t equivalent_steeringrate2 = 0xef;
  const uint8_t equivalent_steerwheelanglelimit = 0x80;
  controllimits_.set_p_adc_cmd_throttlecommandlimit(
      data, adc_cmd_throttlecommandlimit);
  controllimits_.set_p_adc_cmd_steeringrate(data, adc_cmd_steeringrate);
  controllimits_.set_p_adc_cmd_steerwheelanglelimit(
      data, adc_cmd_steerwheelanglelimit);
  EXPECT_EQ(data[3], equivalent_throttlecommandlimit);
  EXPECT_EQ(data[0], equivalent_steeringrate1);
  EXPECT_EQ(data[1], equivalent_steeringrate2);
  EXPECT_EQ(data[2], equivalent_steerwheelanglelimit);
}

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
