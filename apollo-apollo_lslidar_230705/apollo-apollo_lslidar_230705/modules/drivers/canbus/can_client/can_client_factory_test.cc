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

#include "modules/drivers/canbus/can_client/can_client_factory.h"

#include "gtest/gtest.h"

#include "modules/common_msgs/drivers_msgs/can_card_parameter.pb.h"

namespace apollo {
namespace drivers {
namespace canbus {

TEST(CanClientFactoryTest, CreateCanClient) {
  auto can_factory = CanClientFactory::Instance();
  EXPECT_NE(can_factory, nullptr);

  can_factory->RegisterCanClients();

#if USE_ESD_CAN
  CANCardParameter can_card_parameter;
  can_card_parameter.set_brand(CANCardParameter::ESD_CAN);
  can_card_parameter.set_type(CANCardParameter::PCI_CARD);
  can_card_parameter.set_channel_id(CANCardParameter::CHANNEL_ID_ZERO);

  EXPECT_NE(can_factory->CreateCANClient(can_card_parameter), nullptr);
#endif
}

}  // namespace canbus
}  // namespace drivers
}  // namespace apollo
