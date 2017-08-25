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

#include "modules/drivers/can_comm/can_receiver.h"

#include "gtest/gtest.h"

#include "modules/canbus/can_client/fake/fake_can_client.h"
#include "modules/common/proto/error_code.pb.h"
#include "modules/drivers/mobileye_message_manager.h"
#include "modules/drivers/proto/mobileye.pb.h"
#include "modules/drivers/sensor_message_manager.h"

namespace apollo {
namespace drivers {

TEST(CanReceiverTest, ReceiveOne) {
  ::apollo::canbus::can::FakeCanClient can_client;
  SensorMessageManager<Mobileye> pm;
  CanReceiver<Mobileye> receiver;

  receiver.Init(&can_client, &pm, false);
  EXPECT_EQ(receiver.Start(), ::apollo::common::ErrorCode::OK);
  EXPECT_TRUE(receiver.IsRunning());
  receiver.Stop();
  EXPECT_FALSE(receiver.IsRunning());
}

}  // namespace drivers
}  // namespace apollo
