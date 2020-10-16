/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/vehicle/neolix_edu/neolix_edu_message_manager.h"
#include "gtest/gtest.h"
#include "modules/canbus/vehicle/neolix_edu/protocol/ads_brake_command_46.h"
#include "modules/canbus/vehicle/neolix_edu/protocol/ads_diagnosis_628.h"
#include "modules/canbus/vehicle/neolix_edu/protocol/ads_drive_command_50.h"
#include "modules/canbus/vehicle/neolix_edu/protocol/ads_eps_command_56.h"
#include "modules/canbus/vehicle/neolix_edu/protocol/ads_light_horn_command_310.h"

#include "modules/canbus/vehicle/neolix_edu/protocol/aeb_diagnosis1_626.h"
#include "modules/canbus/vehicle/neolix_edu/protocol/aeb_diagresp_718.h"
#include "modules/canbus/vehicle/neolix_edu/protocol/aeb_frontwheelspeed_353.h"
#include "modules/canbus/vehicle/neolix_edu/protocol/aeb_rearwheelspeed_354.h"
#include "modules/canbus/vehicle/neolix_edu/protocol/aeb_systemstate_11.h"
#include "modules/canbus/vehicle/neolix_edu/protocol/aeb_wheelimpulse_355.h"
#include "modules/canbus/vehicle/neolix_edu/protocol/pas_1st_data_311.h"
#include "modules/canbus/vehicle/neolix_edu/protocol/pas_2nd_data_312.h"
#include "modules/canbus/vehicle/neolix_edu/protocol/vcu_brake_report_47.h"
#include "modules/canbus/vehicle/neolix_edu/protocol/vcu_drive_report_52.h"
#include "modules/canbus/vehicle/neolix_edu/protocol/vcu_eps_report_57.h"
#include "modules/canbus/vehicle/neolix_edu/protocol/vcu_nm_401.h"
#include "modules/canbus/vehicle/neolix_edu/protocol/vcu_powerstatus_214.h"
#include "modules/canbus/vehicle/neolix_edu/protocol/vcu_vehicle_fault_response_201.h"
#include "modules/canbus/vehicle/neolix_edu/protocol/vcu_vehicle_info_response_502.h"
#include "modules/canbus/vehicle/neolix_edu/protocol/vcu_vehicle_status_report_101.h"

namespace apollo {
namespace canbus {
namespace neolix_edu {
using ::apollo::canbus::ChassisDetail;
using ::apollo::drivers::canbus::ProtocolData;

class Neolix_eduMessageManagerTest : public ::testing::Test {
 public:
  virtual void SetUp() {}
};
// Control Messages
TEST_F(Neolix_eduMessageManagerTest, Adsbrakecommand46) {
  Neolix_eduMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Adsbrakecommand46::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Adsbrakecommand46 *>(pd)->ID, Adsbrakecommand46::ID);
}
TEST_F(Neolix_eduMessageManagerTest, Adsdiagnosis628) {
  Neolix_eduMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Adsdiagnosis628::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Adsdiagnosis628 *>(pd)->ID, Adsdiagnosis628::ID);
}
TEST_F(Neolix_eduMessageManagerTest, Adsdrivecommand50) {
  Neolix_eduMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Adsdrivecommand50::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Adsdrivecommand50 *>(pd)->ID, Adsdrivecommand50::ID);
}
TEST_F(Neolix_eduMessageManagerTest, Adsepscommand56) {
  Neolix_eduMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Adsepscommand56::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Adsepscommand56 *>(pd)->ID, Adsepscommand56::ID);
}
TEST_F(Neolix_eduMessageManagerTest, Adslighthorncommand310) {
  Neolix_eduMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Adslighthorncommand310::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Adslighthorncommand310 *>(pd)->ID,
            Adslighthorncommand310::ID);
}

// Report Messages
TEST_F(Neolix_eduMessageManagerTest, Aebsystemstate11) {
  Neolix_eduMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Aebsystemstate11::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Aebsystemstate11 *>(pd)->ID, Aebsystemstate11::ID);
}
TEST_F(Neolix_eduMessageManagerTest, Vcubrakereport47) {
  Neolix_eduMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Vcubrakereport47::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Vcubrakereport47 *>(pd)->ID, Vcubrakereport47::ID);
}
TEST_F(Neolix_eduMessageManagerTest, Vcudrivereport52) {
  Neolix_eduMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Vcudrivereport52::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Vcudrivereport52 *>(pd)->ID, Vcudrivereport52::ID);
}
TEST_F(Neolix_eduMessageManagerTest, Vcuepsreport57) {
  Neolix_eduMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Vcuepsreport57::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Vcuepsreport57 *>(pd)->ID, Vcuepsreport57::ID);
}
TEST_F(Neolix_eduMessageManagerTest, Vcunm401) {
  Neolix_eduMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Vcunm401::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Vcunm401 *>(pd)->ID, Vcunm401::ID);
}
TEST_F(Neolix_eduMessageManagerTest, Vcupowerstatus214) {
  Neolix_eduMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Vcupowerstatus214::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Vcupowerstatus214 *>(pd)->ID, Vcupowerstatus214::ID);
}
TEST_F(Neolix_eduMessageManagerTest, Vcuvehiclefaultresponse201) {
  Neolix_eduMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Vcuvehiclefaultresponse201::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Vcuvehiclefaultresponse201 *>(pd)->ID,
            Vcuvehiclefaultresponse201::ID);
}
TEST_F(Neolix_eduMessageManagerTest, Vcuvehicleinforesponse502) {
  Neolix_eduMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Vcuvehicleinforesponse502::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Vcuvehicleinforesponse502 *>(pd)->ID,
            Vcuvehicleinforesponse502::ID);
}
TEST_F(Neolix_eduMessageManagerTest, Vcuvehiclestatusreport101) {
  Neolix_eduMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Vcuvehiclestatusreport101::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Vcuvehiclestatusreport101 *>(pd)->ID,
            Vcuvehiclestatusreport101::ID);
}
TEST_F(Neolix_eduMessageManagerTest, Aebdiagnosis1626) {
  Neolix_eduMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Aebdiagnosis1626::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Aebdiagnosis1626 *>(pd)->ID, Aebdiagnosis1626::ID);
}
TEST_F(Neolix_eduMessageManagerTest, Aebdiagresp718) {
  Neolix_eduMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Aebdiagresp718::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Aebdiagresp718 *>(pd)->ID, Aebdiagresp718::ID);
}
TEST_F(Neolix_eduMessageManagerTest, Aebfrontwheelspeed353) {
  Neolix_eduMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Aebfrontwheelspeed353::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Aebfrontwheelspeed353 *>(pd)->ID,
            Aebfrontwheelspeed353::ID);
}
TEST_F(Neolix_eduMessageManagerTest, Aebrearwheelspeed354) {
  Neolix_eduMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Aebrearwheelspeed354::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Aebrearwheelspeed354 *>(pd)->ID,
            Aebrearwheelspeed354::ID);
}
TEST_F(Neolix_eduMessageManagerTest, Aebwheelimpulse355) {
  Neolix_eduMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Aebwheelimpulse355::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Aebwheelimpulse355 *>(pd)->ID, Aebwheelimpulse355::ID);
}
TEST_F(Neolix_eduMessageManagerTest, Pas1stdata311) {
  Neolix_eduMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Pas1stdata311::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Pas1stdata311 *>(pd)->ID, Pas1stdata311::ID);
}
TEST_F(Neolix_eduMessageManagerTest, Pas2nddata312) {
  Neolix_eduMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Pas2nddata312::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Pas2nddata312 *>(pd)->ID, Pas2nddata312::ID);
}

}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
