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

#include "modules/canbus_vehicle/wey/wey_message_manager.h"
#include "gtest/gtest.h"
#include "modules/canbus_vehicle/wey/protocol/ads1_111.h"
#include "modules/canbus_vehicle/wey/protocol/ads3_38e.h"
#include "modules/canbus_vehicle/wey/protocol/ads_eps_113.h"
#include "modules/canbus_vehicle/wey/protocol/ads_req_vin_390.h"
#include "modules/canbus_vehicle/wey/protocol/ads_shifter_115.h"

#include "modules/canbus_vehicle/wey/protocol/fail_241.h"
#include "modules/canbus_vehicle/wey/protocol/fbs1_243.h"
#include "modules/canbus_vehicle/wey/protocol/fbs2_240.h"
#include "modules/canbus_vehicle/wey/protocol/fbs3_237.h"
#include "modules/canbus_vehicle/wey/protocol/fbs4_235.h"
#include "modules/canbus_vehicle/wey/protocol/status_310.h"
#include "modules/canbus_vehicle/wey/protocol/vin_resp1_391.h"
#include "modules/canbus_vehicle/wey/protocol/vin_resp2_392.h"
#include "modules/canbus_vehicle/wey/protocol/vin_resp3_393.h"

namespace apollo {
namespace canbus {
namespace wey {

class WeyMessageManagerTest : public ::testing::Test {
 public:
  WeyMessageManagerTest() : manager_() {}
  virtual void SetUp() {}

 protected:
  WeyMessageManager manager_;
};

TEST_F(WeyMessageManagerTest, GetSendProtocols) {
  EXPECT_NE(manager_.GetMutableProtocolDataById(Ads1111::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Ads338e::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Adseps113::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Adsreqvin390::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Adsshifter115::ID), nullptr);
}

TEST_F(WeyMessageManagerTest, GetRecvProtocols) {
  EXPECT_NE(manager_.GetMutableProtocolDataById(Fail241::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Fbs1243::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Fbs2240::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Fbs3237::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Fbs4235::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Status310::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Vinresp1391::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Vinresp2392::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(Vinresp3393::ID), nullptr);
}

}  // namespace wey
}  // namespace canbus
}  // namespace apollo
