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

/**
 * @file sensor_message_manager.h
 * @brief The class of SensorMessageManager
 */
#ifndef MODULES_DRIVERS_DELPHI_ESR_DELPHI_ESR_MESSAGE_MANAGER_H_
#define MODULES_DRIVERS_DELPHI_ESR_DELPHI_ESR_MESSAGE_MANAGER_H_

#include "modules/drivers/proto/delphi_esr.pb.h"
#include "modules/drivers/sensor_message_manager.h"
#include "modules/drivers/delphi_esr/protocol/test_4e0.h"

namespace apollo {
namespace drivers {

template <>
SensorMessageManager<DelphiESR>::SensorMessageManager() {
  AddRecvProtocolData<::apollo::drivers::delphi_esr::Test4e0, true>();
}

}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_DELPHI_ESR_DELPHI_ESR_MESSAGE_MANAGER_H_
