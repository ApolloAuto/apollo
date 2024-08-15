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

#include "modules/canbus/vehicle/abstract_vehicle_factory.h"

namespace apollo {
namespace canbus {

void AbstractVehicleFactory::UpdateHeartbeat() {}

void AbstractVehicleFactory::PublishChassisDetailSender() {}

bool AbstractVehicleFactory::CheckChassisCommunicationFault() { return false; }

void AbstractVehicleFactory::AddSendProtocol() {}

void AbstractVehicleFactory::ClearSendProtocol() {}

bool AbstractVehicleFactory::IsSendProtocolClear() { return false; }

Chassis::DrivingMode AbstractVehicleFactory::Driving_Mode() {
  return Chassis::COMPLETE_MANUAL;
}

void AbstractVehicleFactory::SetVehicleParameter(
    const VehicleParameter &vehicle_parameter) {
  vehicle_parameter_ = vehicle_parameter;
}

}  // namespace canbus
}  // namespace apollo
