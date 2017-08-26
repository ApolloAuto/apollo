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

#include "modules/drivers/mobileye/mobileye_message_manager.h"

#include <memory>
#include <set>

#include "gtest/gtest.h"

#include "modules/drivers/sensor_protocol_data.h"
#include "modules/drivers/proto/mobileye.pb.h"

namespace apollo {
namespace drivers {

using apollo::common::ErrorCode;

TEST(MessageManagerTest, TestProtocolDetails738) {
  ::apollo::drivers::mobileye::Details738 details738;
  SensorMessageManager<Mobileye> manager;

  int32_t length = 8;
  uint8_t bytes[8];

  bytes[0] = 0b00000010;
  bytes[1] = 0b00000101;
  bytes[2] = 0b00000101;
  bytes[3] = 0b11111111;
  bytes[4] = 0b00000001;
  bytes[5] = 0b00000011;

  manager.Parse(details738.ID, bytes, length);
  Mobileye mobileye;
  EXPECT_EQ(manager.GetSensorData(&mobileye), ErrorCode::OK);

  EXPECT_EQ(mobileye.details_738().num_obstacles(), 2);
  EXPECT_EQ(mobileye.details_738().go(), 15);
}

}  // namespace drivers 
}  // namespace apollo
