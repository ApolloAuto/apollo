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

#include "modules/canbus_vehicle/transit/protocol/llc_diag_fault_620.h"

#include "gtest/gtest.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace transit {

using ::apollo::drivers::canbus::Byte;

class Diag_fault_620_test : public ::testing::Test {
 public:
  virtual void SetUp() {}

 protected:
  Llcdiagfault620 diagfault_;
};

TEST_F(Diag_fault_620_test, General) {
  const uint8_t bytes[8] = {0x8f, 0x9e, 0xad, 0xbc, 0xcb, 0xda, 0xe9, 0xf8};
  const int32_t length_disengagecounter_brake = 8;
  const int32_t length_disengagecounter_steer = 8;
  const int32_t length_disengagecounter_throttle = 8;
  const int32_t length_fbk_faultcounter = 8;
  const int32_t length_disengagecounter_button = 8;
  const int32_t length_fbk_version_year = 7;
  const int32_t length_fbk_version_month = 4;
  const int32_t length_fbk_version_day = 5;
  const int32_t length_fbk_version_hour = 5;
  const int equivalent_disengagecounter_brake = 0xcb;
  const int equivalent_disengagecounter_steer = 0xad;
  const int equivalent_disengagecounter_throttle = 0xbc;
  const int equivalent_fbk_faultcounter = 0x8f;
  const int equivalent_disengagecounter_button = 0x9e;
  const int equivalent_fbk_version_year = 0x83b;
  const int equivalent_fbk_version_month = 0x03;
  const int equivalent_fbk_version_day = 0x1d;
  const int equivalent_fbk_version_hour = 0x18;
  EXPECT_EQ(diagfault_.llc_disengagecounter_brake(
                bytes, length_disengagecounter_brake),
            equivalent_disengagecounter_brake);
  EXPECT_EQ(diagfault_.llc_disengagecounter_steer(
                bytes, length_disengagecounter_steer),
            equivalent_disengagecounter_steer);
  EXPECT_EQ(diagfault_.llc_disengagecounter_throttle(
                bytes, length_disengagecounter_throttle),
            equivalent_disengagecounter_throttle);
  EXPECT_EQ(diagfault_.llc_fbk_faultcounter(bytes, length_fbk_faultcounter),
            equivalent_fbk_faultcounter);
  EXPECT_EQ(diagfault_.llc_disengagecounter_button(
                bytes, length_disengagecounter_button),
            equivalent_disengagecounter_button);
  EXPECT_EQ(diagfault_.llc_fbk_version_year(bytes, length_fbk_version_year),
            equivalent_fbk_version_year);
  EXPECT_EQ(diagfault_.llc_fbk_version_month(bytes, length_fbk_version_month),
            equivalent_fbk_version_month);
  EXPECT_EQ(diagfault_.llc_fbk_version_day(bytes, length_fbk_version_day),
            equivalent_fbk_version_day);
  EXPECT_EQ(diagfault_.llc_fbk_version_hour(bytes, length_fbk_version_hour),
            equivalent_fbk_version_hour);
}

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
