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

#include "modules/canbus/vehicle/lincoln/protocol/turnsignal_68.h"

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace lincoln {

TEST(Turnsignal68Test, General) {
  Turnsignal68 turn_signal;
  uint8_t data = 0x00;
  turn_signal.UpdateData(&data);
  EXPECT_EQ(turn_signal.turn_cmd(), 0);
  EXPECT_EQ(turn_signal.GetPeriod(), 50000);
  turn_signal.set_turn_none();
  EXPECT_EQ(turn_signal.turn_cmd(), 0x00);
  turn_signal.set_turn_left();
  EXPECT_EQ(turn_signal.turn_cmd(), 0x01);
  turn_signal.set_turn_right();
  EXPECT_EQ(turn_signal.turn_cmd(), 0x02);
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
