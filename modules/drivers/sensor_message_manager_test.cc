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

#include "modules/drivers/sensor_message_manager.h"

#include <memory>
#include <set>

#include "gtest/gtest.h"

#include "modules/drivers/sensor_protocol_data.h"
#include "modules/drivers/proto/mobileye.pb.h"

namespace apollo {
namespace drivers {

using apollo::common::ErrorCode;

class MockProtocolData : public SensorProtocolData<Mobileye> {
 public:
  static const int32_t ID = 0x111;
  MockProtocolData() {}
};

class MockMessageManager : public SensorMessageManager<Mobileye> {
 public:
  MockMessageManager() {
    AddRecvProtocolData<MockProtocolData, true>();
  }
};

TEST(MessageManagerTest, GetMutableProtocolDataById) {
  uint8_t mock_data = 1;
  MockMessageManager manager;
  manager.Parse(MockProtocolData::ID, &mock_data, 8);
  EXPECT_TRUE(manager.GetMutableSensorProtocolDataById(MockProtocolData::ID) !=
              nullptr);

  Mobileye mobileye;
  EXPECT_EQ(manager.GetSensorData(&mobileye), ErrorCode::OK);
  EXPECT_EQ(manager.GetSensorData(nullptr), ErrorCode::CANBUS_ERROR);
}

}  // namespace drivers 
}  // namespace apollo
