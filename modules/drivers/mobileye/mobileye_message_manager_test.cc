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

#include "modules/drivers/proto/mobileye.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace drivers {
namespace canbus {

using apollo::common::ErrorCode;
using apollo::drivers::Mobileye;

TEST(MessageManagerTest, TestProtocolDetails738) {
  ::apollo::drivers::mobileye::Details738 details738;
  MessageManager<Mobileye> manager;

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

TEST(MessageManagerTest, TestProtocolDetails) {
  MessageManager<Mobileye> manager;

  int32_t length = 8;
  uint8_t bytes[8];

  bytes[0] = 0b00000010;
  bytes[1] = 0b00000101;
  bytes[2] = 0b00000101;
  bytes[3] = 0b11111111;
  bytes[4] = 0b00000001;
  bytes[5] = 0b00000011;

  manager.Parse(0x738, bytes, length);

  bytes[0] = 0b00000001;
  bytes[1] = 0b10000000;
  bytes[2] = 0b00000001;
  bytes[3] = 0b01000000;
  bytes[4] = 0b00000010;
  bytes[5] = 0b00000000;
  bytes[6] = 0b00110000;
  bytes[7] = 0b00000011;

  manager.Parse(0x739, bytes, length);

  bytes[0] = 0b00010001;
  bytes[1] = 0b10000100;
  bytes[2] = 0b00000011;
  bytes[3] = 0b00000000;
  bytes[4] = 0b00000000;
  bytes[5] = 0b00000000;
  bytes[6] = 0b00000000;
  bytes[7] = 0b00000000;

  manager.Parse(0x73a, bytes, length);

  bytes[0] = 0b00000000;
  bytes[1] = 0b00000000;
  bytes[2] = 0b00000000;
  bytes[3] = 0b00000000;
  bytes[4] = 0b00000000;
  bytes[5] = 0b00010000;
  bytes[6] = 0b00110000;
  bytes[7] = 0b00000111;

  manager.Parse(0x73b, bytes, length);

  bytes[0] = 0b00000001;
  bytes[1] = 0b10000000;
  bytes[2] = 0b00000001;
  bytes[3] = 0b01000000;
  bytes[4] = 0b00000010;
  bytes[5] = 0b00000000;
  bytes[6] = 0b00110000;
  bytes[7] = 0b00000011;

  manager.Parse(0x73c, bytes, length);

  bytes[0] = 0b00010001;
  bytes[1] = 0b10000100;
  bytes[2] = 0b00000011;
  bytes[3] = 0b00000000;
  bytes[4] = 0b00000000;
  bytes[5] = 0b00000000;
  bytes[6] = 0b00000000;
  bytes[7] = 0b00000000;

  manager.Parse(0x73d, bytes, length);

  bytes[0] = 0b00000000;
  bytes[1] = 0b00000000;
  bytes[2] = 0b00000000;
  bytes[3] = 0b00000000;
  bytes[4] = 0b00000000;
  bytes[5] = 0b00010000;
  bytes[6] = 0b00110000;
  bytes[7] = 0b00000111;

  manager.Parse(0x73e, bytes, length);

  Mobileye mobileye;
  EXPECT_EQ(manager.GetSensorData(&mobileye), ErrorCode::OK);

  EXPECT_EQ(mobileye.details_738().num_obstacles(), 2);
  EXPECT_EQ(mobileye.details_738().go(), 15);

  EXPECT_EQ(2, mobileye.details_739_size());
  EXPECT_EQ(2, mobileye.details_73a_size());
  EXPECT_EQ(2, mobileye.details_73b_size());

  EXPECT_EQ(1, mobileye.details_739(0).obstacle_id());
  EXPECT_NEAR(24.0, mobileye.details_739(0).obstacle_pos_x(), 1e-6);
  EXPECT_NEAR(-28.0, mobileye.details_739(0).obstacle_pos_y(), 1e-6);
  EXPECT_EQ(3, mobileye.details_739(0).obstacle_type());
  EXPECT_EQ(3, mobileye.details_739(0).obstacle_status());

  EXPECT_NEAR(8.5, mobileye.details_73a(0).obstacle_length(), 1e-6);
  EXPECT_NEAR(6.6, mobileye.details_73a(0).obstacle_width(), 1e-6);
  EXPECT_EQ(3, mobileye.details_73a(0).obstacle_age());

  EXPECT_EQ(true, mobileye.details_73b(0).obstacle_replaced());
  EXPECT_NEAR(18.4, mobileye.details_73b(0).obstacle_angle(), 1e-6);

  EXPECT_EQ(1, mobileye.details_739(1).obstacle_id());
  EXPECT_NEAR(24.0, mobileye.details_739(1).obstacle_pos_x(), 1e-6);
  EXPECT_NEAR(-28.0, mobileye.details_739(1).obstacle_pos_y(), 1e-6);
  EXPECT_EQ(3, mobileye.details_739(1).obstacle_type());
  EXPECT_EQ(3, mobileye.details_739(1).obstacle_status());

  EXPECT_NEAR(8.5, mobileye.details_73a(1).obstacle_length(), 1e-6);
  EXPECT_NEAR(6.6, mobileye.details_73a(1).obstacle_width(), 1e-6);
  EXPECT_EQ(3, mobileye.details_73a(1).obstacle_age());

  EXPECT_EQ(true, mobileye.details_73b(1).obstacle_replaced());
  EXPECT_NEAR(18.4, mobileye.details_73b(1).obstacle_angle(), 1e-6);
}

}  // namespace canbus
}  // namespace drivers
}  // namespace apollo
