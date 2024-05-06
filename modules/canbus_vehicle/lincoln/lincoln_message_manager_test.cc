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

#include "modules/canbus_vehicle/lincoln/lincoln_message_manager.h"

#include "gtest/gtest.h"

#include "modules/canbus_vehicle/lincoln/protocol/accel_6b.h"
#include "modules/canbus_vehicle/lincoln/protocol/brake_60.h"
#include "modules/canbus_vehicle/lincoln/protocol/brake_61.h"
#include "modules/canbus_vehicle/lincoln/protocol/brakeinfo_74.h"
#include "modules/canbus_vehicle/lincoln/protocol/fuellevel_72.h"
#include "modules/canbus_vehicle/lincoln/protocol/gear_66.h"
#include "modules/canbus_vehicle/lincoln/protocol/gear_67.h"
#include "modules/canbus_vehicle/lincoln/protocol/gps_6d.h"
#include "modules/canbus_vehicle/lincoln/protocol/gps_6e.h"
#include "modules/canbus_vehicle/lincoln/protocol/gps_6f.h"
#include "modules/canbus_vehicle/lincoln/protocol/gyro_6c.h"
#include "modules/canbus_vehicle/lincoln/protocol/misc_69.h"
#include "modules/canbus_vehicle/lincoln/protocol/steering_64.h"
#include "modules/canbus_vehicle/lincoln/protocol/steering_65.h"
#include "modules/canbus_vehicle/lincoln/protocol/throttle_62.h"
#include "modules/canbus_vehicle/lincoln/protocol/throttle_63.h"
#include "modules/canbus_vehicle/lincoln/protocol/throttleinfo_75.h"
#include "modules/canbus_vehicle/lincoln/protocol/tirepressure_71.h"
#include "modules/canbus_vehicle/lincoln/protocol/turnsignal_68.h"
#include "modules/canbus_vehicle/lincoln/protocol/version_7f.h"
#include "modules/canbus_vehicle/lincoln/protocol/wheelspeed_6a.h"

namespace apollo {
namespace canbus {
namespace lincoln {

using ::apollo::canbus::Lincoln;
using ::apollo::drivers::canbus::ProtocolData;

class LincolnMessageManagerTest : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(LincolnMessageManagerTest, Accel6b) {
  LincolnMessageManager manager;
  ProtocolData<Lincoln> *pd = manager.GetMutableProtocolDataById(Accel6b::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Accel6b *>(pd)->ID, Accel6b::ID);
}

TEST_F(LincolnMessageManagerTest, Brake60) {
  LincolnMessageManager manager;
  ProtocolData<Lincoln> *pd = manager.GetMutableProtocolDataById(Brake60::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Brake60 *>(pd)->ID, Brake60::ID);
}

TEST_F(LincolnMessageManagerTest, Brake61) {
  LincolnMessageManager manager;
  ProtocolData<Lincoln> *pd = manager.GetMutableProtocolDataById(Brake61::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Brake61 *>(pd)->ID, Brake61::ID);
}

TEST_F(LincolnMessageManagerTest, Brakeinfo74) {
  LincolnMessageManager manager;
  ProtocolData<Lincoln> *pd =
      manager.GetMutableProtocolDataById(Brakeinfo74::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Brakeinfo74 *>(pd)->ID, Brakeinfo74::ID);
}

TEST_F(LincolnMessageManagerTest, Fuellevel72) {
  LincolnMessageManager manager;
  ProtocolData<Lincoln> *pd =
      manager.GetMutableProtocolDataById(Fuellevel72::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Fuellevel72 *>(pd)->ID, Fuellevel72::ID);
}

TEST_F(LincolnMessageManagerTest, Gear66) {
  LincolnMessageManager manager;
  ProtocolData<Lincoln> *pd = manager.GetMutableProtocolDataById(Gear66::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Gear66 *>(pd)->ID, Gear66::ID);
}

TEST_F(LincolnMessageManagerTest, Gear67) {
  LincolnMessageManager manager;
  ProtocolData<Lincoln> *pd = manager.GetMutableProtocolDataById(Gear67::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Gear67 *>(pd)->ID, Gear67::ID);
}

TEST_F(LincolnMessageManagerTest, Gps6d) {
  LincolnMessageManager manager;
  ProtocolData<Lincoln> *pd = manager.GetMutableProtocolDataById(Gps6d::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Gps6d *>(pd)->ID, Gps6d::ID);
}

TEST_F(LincolnMessageManagerTest, Gps6e) {
  LincolnMessageManager manager;
  ProtocolData<Lincoln> *pd = manager.GetMutableProtocolDataById(Gps6e::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Gps6e *>(pd)->ID, Gps6e::ID);
}

TEST_F(LincolnMessageManagerTest, Gps6f) {
  LincolnMessageManager manager;
  ProtocolData<Lincoln> *pd = manager.GetMutableProtocolDataById(Gps6f::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Gps6f *>(pd)->ID, Gps6f::ID);
}

TEST_F(LincolnMessageManagerTest, Gyro6c) {
  LincolnMessageManager manager;
  ProtocolData<Lincoln> *pd = manager.GetMutableProtocolDataById(Gyro6c::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Gyro6c *>(pd)->ID, Gyro6c::ID);
}

TEST_F(LincolnMessageManagerTest, Misc69) {
  LincolnMessageManager manager;
  ProtocolData<Lincoln> *pd = manager.GetMutableProtocolDataById(Misc69::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Misc69 *>(pd)->ID, Misc69::ID);
}

TEST_F(LincolnMessageManagerTest, Steering64) {
  LincolnMessageManager manager;
  ProtocolData<Lincoln> *pd =
      manager.GetMutableProtocolDataById(Steering64::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Steering64 *>(pd)->ID, Steering64::ID);
}

TEST_F(LincolnMessageManagerTest, Steering65) {
  LincolnMessageManager manager;
  ProtocolData<Lincoln> *pd =
      manager.GetMutableProtocolDataById(Steering65::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Steering65 *>(pd)->ID, Steering65::ID);
}

TEST_F(LincolnMessageManagerTest, Throttle62) {
  LincolnMessageManager manager;
  ProtocolData<Lincoln> *pd =
      manager.GetMutableProtocolDataById(Throttle62::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Throttle62 *>(pd)->ID, Throttle62::ID);
}

TEST_F(LincolnMessageManagerTest, Throttle63) {
  LincolnMessageManager manager;
  ProtocolData<Lincoln> *pd =
      manager.GetMutableProtocolDataById(Throttle63::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Throttle63 *>(pd)->ID, Throttle63::ID);
}

TEST_F(LincolnMessageManagerTest, Throttleinfo75) {
  LincolnMessageManager manager;
  ProtocolData<Lincoln> *pd =
      manager.GetMutableProtocolDataById(Throttleinfo75::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Throttleinfo75 *>(pd)->ID, Throttleinfo75::ID);
}

TEST_F(LincolnMessageManagerTest, Tirepressure71) {
  LincolnMessageManager manager;
  ProtocolData<Lincoln> *pd =
      manager.GetMutableProtocolDataById(Tirepressure71::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Tirepressure71 *>(pd)->ID, Tirepressure71::ID);
}

TEST_F(LincolnMessageManagerTest, Turnsignal68) {
  LincolnMessageManager manager;
  ProtocolData<Lincoln> *pd =
      manager.GetMutableProtocolDataById(Turnsignal68::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Turnsignal68 *>(pd)->ID, Turnsignal68::ID);
}

TEST_F(LincolnMessageManagerTest, Version7f) {
  LincolnMessageManager manager;
  ProtocolData<Lincoln> *pd = manager.GetMutableProtocolDataById(Version7f::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Version7f *>(pd)->ID, Version7f::ID);
}

TEST_F(LincolnMessageManagerTest, Wheelspeed6a) {
  LincolnMessageManager manager;
  ProtocolData<Lincoln> *pd =
      manager.GetMutableProtocolDataById(Wheelspeed6a::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Wheelspeed6a *>(pd)->ID, Wheelspeed6a::ID);
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
