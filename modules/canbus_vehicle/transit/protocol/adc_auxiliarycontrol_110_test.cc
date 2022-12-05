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

#include "modules/canbus_vehicle/transit/protocol/adc_auxiliarycontrol_110.h"

#include "gtest/gtest.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace transit {

using ::apollo::drivers::canbus::Byte;

class Auxiliarycontrol_110_test : public ::testing::Test {
 public:
  virtual void SetUp() {}

 protected:
  Adcauxiliarycontrol110 control_;
};

TEST_F(Auxiliarycontrol_110_test, General) {
  uint8_t data[8] = {0x8f, 0x9e, 0xad, 0xbc, 0xcb, 0xda, 0xe9, 0xf8};
  const int adc_auxcontrol_counter = 2;
  const int adc_auxcontrol_checksum = 127;
  const bool adc_cmd_inverter_controlenable = 0;
  const bool adc_cmd_inverter = 0;
  const int adc_cmd_wiper = 5;
  const bool adc_cmd_pdu_controlenable = 0;
  const bool adc_cmd_pdu_ch8 = 0;
  const bool adc_cmd_pdu_ch7 = 0;
  const bool adc_cmd_pdu_ch6 = 0;
  const bool adc_cmd_pdu_ch5 = 0;
  const bool adc_cmd_pdu_ch4 = 0;
  const bool adc_cmd_pdu_ch3 = 0;
  const bool adc_cmd_pdu_ch2 = 0;
  const bool adc_cmd_pdu_ch1 = 0;
  const bool adc_cmd_hazardlights = 0;
  const bool adc_cmd_highbeam = 0;
  const bool adc_cmd_lowbeam = 0;
  const bool adc_cmd_horn = 0;
  Adc_auxiliarycontrol_110::Adc_cmd_turnsignalType adc_cmd_turnsignal =
      Adc_auxiliarycontrol_110::ADC_CMD_TURNSIGNAL_RIGHT;
  const uint8_t equivalent_6 = 0xa9;
  const uint8_t equivalent_7 = 0x7f;
  const uint8_t equivalent_0 = 0xb0;
  const uint8_t equivalent_1 = 0x00;
  const uint8_t equivalent_3 = 0xa2;

  control_.set_p_adc_auxcontrol_counter(data, adc_auxcontrol_counter);
  control_.set_p_adc_auxcontrol_checksum(data, adc_auxcontrol_checksum);
  control_.set_p_adc_cmd_inverter_controlenable(data,
                                                adc_cmd_inverter_controlenable);
  control_.set_p_adc_cmd_inverter(data, adc_cmd_inverter);
  control_.set_p_adc_cmd_wiper(data, adc_cmd_wiper);
  control_.set_p_adc_cmd_pdu_controlenable(data, adc_cmd_pdu_controlenable);
  control_.set_p_adc_cmd_pdu_ch8(data, adc_cmd_pdu_ch8);
  control_.set_p_adc_cmd_pdu_ch7(data, adc_cmd_pdu_ch7);
  control_.set_p_adc_cmd_pdu_ch6(data, adc_cmd_pdu_ch6);
  control_.set_p_adc_cmd_pdu_ch5(data, adc_cmd_pdu_ch5);
  control_.set_p_adc_cmd_pdu_ch4(data, adc_cmd_pdu_ch4);
  control_.set_p_adc_cmd_pdu_ch3(data, adc_cmd_pdu_ch3);
  control_.set_p_adc_cmd_pdu_ch2(data, adc_cmd_pdu_ch2);
  control_.set_p_adc_cmd_pdu_ch1(data, adc_cmd_pdu_ch1);
  control_.set_p_adc_cmd_hazardlights(data, adc_cmd_hazardlights);
  control_.set_p_adc_cmd_highbeam(data, adc_cmd_highbeam);
  control_.set_p_adc_cmd_lowbeam(data, adc_cmd_lowbeam);
  control_.set_p_adc_cmd_horn(data, adc_cmd_horn);
  control_.set_p_adc_cmd_turnsignal(data, adc_cmd_turnsignal);

  EXPECT_EQ(data[6], equivalent_6);
  EXPECT_EQ(data[7], equivalent_7);
  EXPECT_EQ(data[0], equivalent_0);
  EXPECT_EQ(data[1], equivalent_1);
  EXPECT_EQ(data[3], equivalent_3);
}

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
