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

#include "modules/canbus/vehicle/ge3/ge3_message_manager.h"
#include "gtest/gtest.h"
#include "modules/canbus/vehicle/ge3/protocol/pc_bcm_201.h"
#include "modules/canbus/vehicle/ge3/protocol/pc_bcs_202.h"
#include "modules/canbus/vehicle/ge3/protocol/pc_epb_203.h"
#include "modules/canbus/vehicle/ge3/protocol/pc_eps_204.h"
#include "modules/canbus/vehicle/ge3/protocol/pc_vcu_205.h"
#include "modules/canbus/vehicle/ge3/protocol/scu_1_301.h"
#include "modules/canbus/vehicle/ge3/protocol/scu_2_302.h"
#include "modules/canbus/vehicle/ge3/protocol/scu_3_303.h"
#include "modules/canbus/vehicle/ge3/protocol/scu_bcm_304.h"
#include "modules/canbus/vehicle/ge3/protocol/scu_bcs_1_306.h"
#include "modules/canbus/vehicle/ge3/protocol/scu_bcs_2_307.h"
#include "modules/canbus/vehicle/ge3/protocol/scu_bcs_3_308.h"
#include "modules/canbus/vehicle/ge3/protocol/scu_epb_310.h"
#include "modules/canbus/vehicle/ge3/protocol/scu_eps_311.h"
#include "modules/canbus/vehicle/ge3/protocol/scu_vcu_1_312.h"
#include "modules/canbus/vehicle/ge3/protocol/scu_vcu_2_313.h"

namespace apollo {
namespace canbus {
namespace ge3 {

class Ge3MessageManagerTest : public ::testing::Test {
 public:
  Ge3MessageManagerTest() : manager_() {}
  virtual void SetUp() {}

 protected:
  Ge3MessageManager manager_;
};

TEST_F(Ge3MessageManagerTest, GetSendProtocols) {
  EXPECT_NE(manager_.GetMutableProtocolDataById(Pcbcm201::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Pcbcs202::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Pcepb203::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Pceps204::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Pcvcu205::ID), nullptr);
}

TEST_F(Ge3MessageManagerTest, GetRecvProtocols) {
  EXPECT_NE(manager_.GetMutableProtocolDataById(Scu1301::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Scu2302::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Scu3303::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Scubcm304::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Scubcs1306::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Scubcs2307::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Scubcs3308::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Scuepb310::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Scueps311::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Scuvcu1312::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Scuvcu2313::ID), nullptr);
}

}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
