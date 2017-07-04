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

#include "modules/canbus/vehicle/lincoln/protocol/brakeinfo_74.h"

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace lincoln {

class Brakeinfo74Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Brakeinfo74Test, simple) {
  Brakeinfo74 brakeinfo;
  uint8_t data = 0x64U;
  int32_t length = 8;
  ChassisDetail cd;
  brakeinfo.Parse(&data, length, &cd);

  EXPECT_FALSE(cd.esp().is_abs_active());
  EXPECT_FALSE(cd.esp().is_abs_enabled());
  EXPECT_FALSE(cd.esp().is_stab_active());
  EXPECT_FALSE(cd.esp().is_stab_enabled());
  EXPECT_FALSE(cd.esp().is_trac_active());
  EXPECT_FALSE(cd.esp().is_trac_enabled());

  EXPECT_EQ(cd.epb().parking_brake_status(), Epb::PBRAKE_OFF);
  EXPECT_EQ(cd.brake().brake_torque_req(), 8592);
  EXPECT_EQ(cd.brake().hsa_status(), Brake::HSA_INACTIVE);
  EXPECT_EQ(cd.brake().brake_torque_act(), 0);
  EXPECT_EQ(cd.brake().hsa_status(), Brake::HSA_OFF);
  EXPECT_EQ(cd.brake().wheel_torque_act(), 8192);
  EXPECT_FALSE(cd.vehicle_spd().is_vehicle_standstill());
  EXPECT_DOUBLE_EQ(cd.vehicle_spd().acc_est(), 0.0);
}

}  // namespace lincoln
}  // namespace apollo
}  // namespace canbus
