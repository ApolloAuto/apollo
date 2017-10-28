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

#include "modules/drivers/delphi_esr/delphi_esr_message_manager.h"

#include <memory>
#include <set>

#include "gtest/gtest.h"

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/drivers/proto/delphi_esr.pb.h"

namespace apollo {
namespace drivers {

using apollo::common::ErrorCode;

TEST(DelphiESRMessageManagerTest, TestProtocolTest4e0) {
  ::apollo::drivers::delphi_esr::Esrstatus14e0 test_4e0;
  canbus::MessageManager<DelphiESR> manager;

  int32_t length = 8;
  uint8_t bytes[8];

  bytes[0] = 0b00000010;
  bytes[1] = 0b00000101;
  bytes[2] = 0b00000101;
  bytes[3] = 0b00000001;
  bytes[4] = 0b00000101;
  bytes[5] = 0b00000011;

  manager.Parse(test_4e0.ID, bytes, length);
  DelphiESR delphi_esr;
  EXPECT_EQ(manager.GetSensorData(&delphi_esr), ErrorCode::OK);

  EXPECT_EQ(delphi_esr.esr_status1_4e0().can_tx_scan_index(), 261);
}

}  // namespace drivers
}  // namespace apollo
