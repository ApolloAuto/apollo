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
 * @file
 */

#ifndef MODULES_DRIVERS_DELPHI_ESR_DELPHI_ESR_CANBUS_H_
#define MODULES_DRIVERS_DELPHI_ESR_DELPHI_ESR_CANBUS_H_

#include "modules/drivers/canbus/sensor_canbus.h"
#include "modules/drivers/delphi_esr/delphi_esr_message_manager.h"
#include "modules/drivers/proto/delphi_esr.pb.h"

/**
 * @namespace apollo::drivers
 * @brief apollo::drivers
 */
namespace apollo {
namespace drivers {

/**
* @class SensorCanbus
*
* @brief template of canbus-based sensor module main class (e.g., delphi_esr).
*/

template <>
void SensorCanbus<DelphiESR>::PublishSensorData() {
  DelphiESR delphi_esr;
  sensor_message_manager_->GetSensorData(&delphi_esr);
  ADEBUG << delphi_esr.ShortDebugString();

  AdapterManager::FillDelphiESRHeader(FLAGS_sensor_node_name, &delphi_esr);
  AdapterManager::PublishDelphiESR(delphi_esr);
}

}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_DELPHI_ESR_DELPHI_ESR_CANBUS_H_
