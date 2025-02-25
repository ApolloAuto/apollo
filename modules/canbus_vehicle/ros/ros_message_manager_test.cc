// Copyright 2025 daohu527@gmail.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//  Created Date: 2025-01-16
//  Author: daohu527

#include "modules/canbus_vehicle/ros/ros_message_manager.h"

#include "gtest/gtest.h"

#include "modules/canbus_vehicle/ros/protocol/acc_fb.h"
#include "modules/canbus_vehicle/ros/protocol/ang_vel_fb.h"
#include "modules/canbus_vehicle/ros/protocol/twist_cmd.h"
#include "modules/canbus_vehicle/ros/protocol/twist_fb.h"

namespace apollo {
namespace canbus {
namespace ros {

class RosMessageManagerTest : public ::testing::Test {
 public:
  virtual void SetUp() {}

 protected:
  RosMessageManager manager_;
};

TEST_F(RosMessageManagerTest, GetSendProtocols) {
  EXPECT_NE(manager_.GetMutableProtocolDataById(TwistCmd::ID), nullptr);
}

TEST_F(RosMessageManagerTest, GetRecvProtocols) {
  EXPECT_NE(manager_.GetMutableProtocolDataById(TwistFb::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(AccFb::ID), nullptr);
  EXPECT_NE(manager_.GetMutableProtocolDataById(AngVelFb::ID), nullptr);
}

}  // namespace ros
}  // namespace canbus
}  // namespace apollo
