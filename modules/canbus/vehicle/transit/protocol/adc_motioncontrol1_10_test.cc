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

#include "modules/canbus/vehicle/transit/protocol/adc_motioncontrol1_10.h"

#include "gtest/gtest.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace transit {
using ::apollo::drivers::canbus::Byte;

class adc_motioncontrol1_10Test : public ::testing ::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(adc_motioncontrol1_10Test, part1) {
  Adcmotioncontrol110 motion_ctrl_110_;
  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  double x = 0.0;
  int i = 0;
  motion_ctrl_110_.set_p_adc_cmd_steerwheelangle(data, x);
  motion_ctrl_110_.set_p_adc_cmd_gear(data, motion_ctrl_110_.adc_cmd_gear_);
  motion_ctrl_110_.set_p_adc_motioncontrol1_checksum(data, i);
  motion_ctrl_110_.set_p_adc_cmd_brakepercentage(data, x);

  EXPECT_EQ(data[0], 0x3F);
  EXPECT_EQ(data[1], 0x0);
  EXPECT_EQ(data[2], 0xFE);
  EXPECT_EQ(data[3], 0x7);
  EXPECT_EQ(data[4], 0x0);
  EXPECT_EQ(data[5], 0xF8);
  EXPECT_EQ(data[6], 0xE3);
  EXPECT_EQ(data[7], 0x0);
}

TEST_F(adc_motioncontrol1_10Test, part2) {
  Adcmotioncontrol110 motion_ctrl_110_;
  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  double x = 0.0;
  bool flag = false;
  motion_ctrl_110_.set_p_adc_cmd_steeringcontrolmode(
      data, motion_ctrl_110_.adc_cmd_steeringcontrolmode_);
  motion_ctrl_110_.set_p_adc_cmd_parkingbrake(data, flag);
  motion_ctrl_110_.set_p_adc_cmd_throttleposition(data, x);

  EXPECT_EQ(data[0], 0xCF);
  EXPECT_EQ(data[1], 0xFF);
  EXPECT_EQ(data[2], 0x01);
  EXPECT_EQ(data[3], 0xF8);
  EXPECT_EQ(data[4], 0xFF);
  EXPECT_EQ(data[5], 0xFF);
  EXPECT_EQ(data[6], 0xDF);
  EXPECT_EQ(data[7], 0xFF);
}

TEST_F(adc_motioncontrol1_10Test, part3) {
  Adcmotioncontrol110 motion_ctrl_110_;
  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  int i = 0;
  motion_ctrl_110_.set_p_adc_motioncontrol1_counter(data, i);
  motion_ctrl_110_.set_p_adc_cmd_autonomyrequest(
      data, motion_ctrl_110_.adc_cmd_autonomyrequest_);

  EXPECT_EQ(data[0], 0xFC);
  EXPECT_EQ(data[1], 0xFF);
  EXPECT_EQ(data[2], 0xFF);
  EXPECT_EQ(data[3], 0xFF);
  EXPECT_EQ(data[4], 0xFF);
  EXPECT_EQ(data[5], 0xFF);
  EXPECT_EQ(data[6], 0x3F);
  EXPECT_EQ(data[7], 0xFF);
}

TEST_F(adc_motioncontrol1_10Test, part4) {
  Adcmotioncontrol110 motion_ctrl_110_;
  uint8_t data = 0xFF;
  motion_ctrl_110_.set_p_adc_cmd_longitudinalcontrolmode(
      &data, motion_ctrl_110_.adc_cmd_longitudinalcontrolmode_);

  EXPECT_EQ(data, 0xF3);
}

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
