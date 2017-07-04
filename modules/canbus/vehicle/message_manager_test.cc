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

#include "modules/canbus/vehicle/message_manager.h"

#include <memory>
#include <set>

#include "gtest/gtest.h"

#include "modules/canbus/vehicle/protocol_data.h"

namespace apollo {
namespace canbus {

using apollo::common::ErrorCode;

class MockProtocolData : public ProtocolData {
 public:
  static const int32_t ID = 0x111;
  MockProtocolData() {}
};

class MockMessageManager : public MessageManager {
 public:
  MockMessageManager() {
    AddRecvProtocolData<MockProtocolData, true>();
    AddSendProtocolData<MockProtocolData, true>();
  }
};

TEST(MessageManagerTest, GetMutableProtocolDataById) {
  struct timeval curr_time;
  uint8_t mock_data = 1;
  MockMessageManager manager;
  manager.Parse(MockProtocolData::ID, &mock_data, 8, curr_time);
  manager.ResetSendMessages();
  EXPECT_TRUE(manager.GetMutableProtocolDataById(MockProtocolData::ID) !=
              nullptr);

  ChassisDetail chassis_detail;
  chassis_detail.set_car_type(ChassisDetail::QIRUI_EQ_15);
  EXPECT_EQ(manager.GetChassisDetail(&chassis_detail), ErrorCode::OK);
  EXPECT_EQ(manager.GetChassisDetail(nullptr), ErrorCode::CANBUS_ERROR);
}

}  // namespace canbus
}  // namespace apollo
