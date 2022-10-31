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

#include "modules/drivers/canbus/can_comm/can_sender.h"

#include "gtest/gtest.h"

#include "modules/common_msgs/chassis_msgs/chassis_detail.pb.h"
#include "modules/common_msgs/basic_msgs/error_code.pb.h"
#include "modules/drivers/canbus/can_client/fake/fake_can_client.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace drivers {
namespace canbus {

TEST(CanSenderTest, OneRunCase) {
  CanSender<::apollo::canbus::ChassisDetail> sender;
  MessageManager<::apollo::canbus::ChassisDetail> pm;
  can::FakeCanClient can_client;
  sender.Init(&can_client, &pm, true);

  ProtocolData<::apollo::canbus::ChassisDetail> mpd;
  SenderMessage<::apollo::canbus::ChassisDetail> msg(1, &mpd);
  EXPECT_FALSE(sender.NeedSend(msg, 1));
  EXPECT_EQ(msg.message_id(), 1);
  int32_t period = msg.curr_period();
  msg.UpdateCurrPeriod(-50);
  EXPECT_EQ(msg.curr_period(), period + 50);
  EXPECT_EQ(msg.CanFrame().id, 1);

  sender.AddMessage(1, &mpd);
  EXPECT_EQ(sender.Start(), common::ErrorCode::OK);
  EXPECT_TRUE(sender.IsRunning());
  EXPECT_TRUE(sender.enable_log());

  sender.Update();
  sender.Stop();
  EXPECT_FALSE(sender.IsRunning());
}

}  // namespace canbus
}  // namespace drivers
}  // namespace apollo
