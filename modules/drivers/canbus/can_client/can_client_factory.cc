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

#include "modules/drivers/canbus/can_client/fake/fake_can_client.h"
#if USE_ESD_CAN
#include "modules/drivers/canbus/can_client/esd/esd_can_client.h"
#endif

#include "modules/drivers/canbus/can_client/socket/socket_can_client_raw.h"

#include "modules/common/log.h"
#include "modules/common/util/util.h"

namespace apollo {
namespace drivers {
namespace canbus {

CanClientFactory::CanClientFactory() {}

void CanClientFactory::RegisterCanClients() {
  AINFO << "CanClientFactory::RegisterCanClients";
  Register(CANCardParameter::FAKE_CAN,
           []() -> CanClient* { return new can::FakeCanClient(); });
#if USE_ESD_CAN
  AINFO << "register can: " << CANCardParameter::ESD_CAN;
  Register(CANCardParameter::ESD_CAN,
           []() -> CanClient* { return new can::EsdCanClient(); });
#endif
  Register(CANCardParameter::SOCKET_CAN_RAW,
           []() -> CanClient* { return new can::SocketCanClientRaw(); });
}

std::unique_ptr<CanClient> CanClientFactory::CreateCANClient(
    const CANCardParameter& parameter) {
  auto factory = CreateObject(parameter.brand());
  if (!factory) {
    AERROR << "Failed to create CAN client with parameter: "
           << parameter.DebugString();
  } else if (!factory->Init(parameter)) {
    AERROR << "Failed to initialize CAN card with parameter: "
           << parameter.DebugString();
  }
  return factory;
}

}  // namespace canbus
}  // namespace drivers
}  // namespace apollo
