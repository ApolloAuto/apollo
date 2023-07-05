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

#include "modules/canbus_vehicle/gem/gem_message_manager.h"

#include "gtest/gtest.h"

#include "modules/canbus_vehicle/gem/protocol/accel_cmd_67.h"
#include "modules/canbus_vehicle/gem/protocol/accel_rpt_68.h"
#include "modules/canbus_vehicle/gem/protocol/brake_cmd_6b.h"
#include "modules/canbus_vehicle/gem/protocol/brake_motor_rpt_1_70.h"
#include "modules/canbus_vehicle/gem/protocol/brake_motor_rpt_2_71.h"
#include "modules/canbus_vehicle/gem/protocol/brake_motor_rpt_3_72.h"
#include "modules/canbus_vehicle/gem/protocol/brake_rpt_6c.h"
#include "modules/canbus_vehicle/gem/protocol/date_time_rpt_83.h"
#include "modules/canbus_vehicle/gem/protocol/global_cmd_69.h"
#include "modules/canbus_vehicle/gem/protocol/global_rpt_6a.h"
#include "modules/canbus_vehicle/gem/protocol/headlight_cmd_76.h"
#include "modules/canbus_vehicle/gem/protocol/headlight_rpt_77.h"
#include "modules/canbus_vehicle/gem/protocol/horn_cmd_78.h"
#include "modules/canbus_vehicle/gem/protocol/horn_rpt_79.h"
#include "modules/canbus_vehicle/gem/protocol/lat_lon_heading_rpt_82.h"
#include "modules/canbus_vehicle/gem/protocol/parking_brake_status_rpt_80.h"
#include "modules/canbus_vehicle/gem/protocol/shift_cmd_65.h"
#include "modules/canbus_vehicle/gem/protocol/shift_rpt_66.h"
#include "modules/canbus_vehicle/gem/protocol/steering_cmd_6d.h"
#include "modules/canbus_vehicle/gem/protocol/steering_motor_rpt_1_73.h"
#include "modules/canbus_vehicle/gem/protocol/steering_motor_rpt_2_74.h"
#include "modules/canbus_vehicle/gem/protocol/steering_motor_rpt_3_75.h"
#include "modules/canbus_vehicle/gem/protocol/steering_rpt_1_6e.h"
#include "modules/canbus_vehicle/gem/protocol/turn_cmd_63.h"
#include "modules/canbus_vehicle/gem/protocol/turn_rpt_64.h"
#include "modules/canbus_vehicle/gem/protocol/vehicle_speed_rpt_6f.h"
#include "modules/canbus_vehicle/gem/protocol/wheel_speed_rpt_7a.h"
#include "modules/canbus_vehicle/gem/protocol/wiper_cmd_90.h"
#include "modules/canbus_vehicle/gem/protocol/wiper_rpt_91.h"
#include "modules/canbus_vehicle/gem/protocol/yaw_rate_rpt_81.h"

namespace apollo {
namespace canbus {
namespace gem {

class GemMessageManagerTest : public ::testing::Test {
 protected:
  GemMessageManager manager_;
};

TEST_F(GemMessageManagerTest, GetSendProtocols) {
  EXPECT_NE(manager_.GetMutableProtocolDataById(Accelcmd67::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Brakecmd6b::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Globalcmd69::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Headlightcmd76::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Horncmd78::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Shiftcmd65::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Steeringcmd6d::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Turncmd63::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Wipercmd90::ID), nullptr);
}

TEST_F(GemMessageManagerTest, GetRecvProtocols) {
  EXPECT_NE(manager_.GetMutableProtocolDataById(Accelrpt68::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Brakemotorrpt170::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Brakemotorrpt271::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Brakemotorrpt372::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Brakerpt6c::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Datetimerpt83::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Globalrpt6a::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Headlightrpt77::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Hornrpt79::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Latlonheadingrpt82::ID),
            nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Parkingbrakestatusrpt80::ID),
            nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Shiftrpt66::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Steeringmotorrpt173::ID),
            nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Steeringmotorrpt274::ID),
            nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Steeringmotorrpt375::ID),
            nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Steeringrpt16e::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Turnrpt64::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Vehiclespeedrpt6f::ID),
            nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Wheelspeedrpt7a::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Wiperrpt91::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Yawraterpt81::ID), nullptr);
}

}  // namespace gem
}  // namespace canbus
}  // namespace apollo
