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

#include "modules/canbus_vehicle/ch/ch_message_manager.h"
#include "gtest/gtest.h"
#include "modules/canbus_vehicle/ch/protocol/brake_command_111.h"
#include "modules/canbus_vehicle/ch/protocol/brake_status__511.h"
#include "modules/canbus_vehicle/ch/protocol/ecu_status_1_515.h"
#include "modules/canbus_vehicle/ch/protocol/ecu_status_2_516.h"
#include "modules/canbus_vehicle/ch/protocol/ecu_status_3_517.h"
#include "modules/canbus_vehicle/ch/protocol/gear_command_114.h"
#include "modules/canbus_vehicle/ch/protocol/gear_status_514.h"
#include "modules/canbus_vehicle/ch/protocol/steer_command_112.h"
#include "modules/canbus_vehicle/ch/protocol/steer_status__512.h"
#include "modules/canbus_vehicle/ch/protocol/throttle_command_110.h"
#include "modules/canbus_vehicle/ch/protocol/throttle_status__510.h"
#include "modules/canbus_vehicle/ch/protocol/turnsignal_command_113.h"
#include "modules/canbus_vehicle/ch/protocol/turnsignal_status__513.h"

namespace apollo {
namespace canbus {
namespace ch {
using ::apollo::canbus::Ch;
using ::apollo::drivers::canbus::ProtocolData;

class ChMessageManagerTest : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(ChMessageManagerTest, Brakecommand111) {
  ChMessageManager manager;
  ProtocolData<Ch> *pd =
      manager.GetMutableProtocolDataById(Brakecommand111::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Brakecommand111 *>(pd)->ID, Brakecommand111::ID);
}

TEST_F(ChMessageManagerTest, Brakestatus511) {
  ChMessageManager manager;
  ProtocolData<Ch> *pd =
      manager.GetMutableProtocolDataById(Brakestatus511::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Brakestatus511 *>(pd)->ID, Brakestatus511::ID);
}

TEST_F(ChMessageManagerTest, Ecustatus1515) {
  ChMessageManager manager;
  ProtocolData<Ch> *pd =
      manager.GetMutableProtocolDataById(Ecustatus1515::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Ecustatus1515 *>(pd)->ID, Ecustatus1515::ID);
}

TEST_F(ChMessageManagerTest, Ecustatus2516) {
  ChMessageManager manager;
  ProtocolData<Ch> *pd =
      manager.GetMutableProtocolDataById(Ecustatus2516::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Ecustatus2516 *>(pd)->ID, Ecustatus2516::ID);
}

TEST_F(ChMessageManagerTest, Ecustatus3517) {
  ChMessageManager manager;
  ProtocolData<Ch> *pd =
      manager.GetMutableProtocolDataById(Ecustatus3517::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Ecustatus3517 *>(pd)->ID, Ecustatus3517::ID);
}

TEST_F(ChMessageManagerTest, Gearcommand114) {
  ChMessageManager manager;
  ProtocolData<Ch> *pd =
      manager.GetMutableProtocolDataById(Gearcommand114::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Gearcommand114 *>(pd)->ID, Gearcommand114::ID);
}

TEST_F(ChMessageManagerTest, Gearstatus514) {
  ChMessageManager manager;
  ProtocolData<Ch> *pd =
      manager.GetMutableProtocolDataById(Gearstatus514::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Gearstatus514 *>(pd)->ID, Gearstatus514::ID);
}

TEST_F(ChMessageManagerTest, Steercommand112) {
  ChMessageManager manager;
  ProtocolData<Ch> *pd =
      manager.GetMutableProtocolDataById(Steercommand112::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Steercommand112 *>(pd)->ID, Steercommand112::ID);
}

TEST_F(ChMessageManagerTest, Steerstatus512) {
  ChMessageManager manager;
  ProtocolData<Ch> *pd =
      manager.GetMutableProtocolDataById(Steerstatus512::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Steerstatus512 *>(pd)->ID, Steerstatus512::ID);
}

TEST_F(ChMessageManagerTest, Throttlecommand110) {
  ChMessageManager manager;
  ProtocolData<Ch> *pd =
      manager.GetMutableProtocolDataById(Throttlecommand110::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Throttlecommand110 *>(pd)->ID, Throttlecommand110::ID);
}

TEST_F(ChMessageManagerTest, Throttlestatus510) {
  ChMessageManager manager;
  ProtocolData<Ch> *pd =
      manager.GetMutableProtocolDataById(Throttlestatus510::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Throttlestatus510 *>(pd)->ID, Throttlestatus510::ID);
}

TEST_F(ChMessageManagerTest, Turnsignalcommand113) {
  ChMessageManager manager;
  ProtocolData<Ch> *pd =
      manager.GetMutableProtocolDataById(Turnsignalcommand113::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Turnsignalcommand113 *>(pd)->ID,
            Turnsignalcommand113::ID);
}

TEST_F(ChMessageManagerTest, Turnsignalstatus513) {
  ChMessageManager manager;
  ProtocolData<Ch> *pd =
      manager.GetMutableProtocolDataById(Turnsignalstatus513::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Turnsignalstatus513 *>(pd)->ID,
            Turnsignalstatus513::ID);
}

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
