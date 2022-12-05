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

#include "modules/canbus_vehicle/lincoln/protocol/misc_69.h"

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace lincoln {

TEST(Misc69Test, General) {
  uint8_t data[8] = {0x67, 0x62, 0x63, 0x64, 0x51, 0x52, 0x53, 0x54};
  int32_t length = 8;
  Lincoln cd;
  Misc69 misc;
  misc.Parse(data, length, &cd);

  EXPECT_FALSE(cd.basic().acc_on_button());
  EXPECT_FALSE(cd.basic().acc_off_button());
  EXPECT_TRUE(cd.basic().acc_res_button());
  EXPECT_TRUE(cd.basic().acc_cancel_button());
  EXPECT_TRUE(cd.basic().acc_on_off_button());
  EXPECT_TRUE(cd.basic().acc_res_cancel_button());
  EXPECT_FALSE(cd.basic().acc_inc_spd_button());
  EXPECT_FALSE(cd.basic().acc_dec_spd_button());
  EXPECT_FALSE(cd.basic().acc_inc_gap_button());
  EXPECT_TRUE(cd.basic().acc_dec_gap_button());
  EXPECT_TRUE(cd.basic().lka_button());
  EXPECT_FALSE(cd.basic().canbus_fault());

  EXPECT_TRUE(cd.safety().is_driver_car_door_close());
  EXPECT_TRUE(cd.safety().is_driver_buckled());
  EXPECT_FALSE(cd.safety().is_passenger_door_open());
  EXPECT_TRUE(cd.safety().is_rearleft_door_open());
  EXPECT_FALSE(cd.safety().is_rearright_door_open());
  EXPECT_FALSE(cd.safety().is_hood_open());
  EXPECT_TRUE(cd.safety().is_trunk_open());
  EXPECT_TRUE(cd.safety().is_passenger_detected());
  EXPECT_FALSE(cd.safety().is_passenger_airbag_enabled());
  EXPECT_FALSE(cd.safety().is_passenger_buckled());

  EXPECT_EQ(cd.light().lincoln_lamp_type(), Light::BEAM_FLASH_TO_PASS);
  EXPECT_EQ(cd.light().lincoln_wiper(), Light::WIPER_MANUAL_HIGH);
  EXPECT_EQ(cd.light().lincoln_ambient(), Light::AMBIENT_TWILIGHT);
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
