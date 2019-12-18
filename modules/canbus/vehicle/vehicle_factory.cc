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

#include "modules/canbus/vehicle/vehicle_factory.h"
#include "modules/canbus/proto/vehicle_parameter.pb.h"
#include "modules/canbus/vehicle/ch/ch_vehicle_factory.h"
#include "modules/canbus/vehicle/ge3/ge3_vehicle_factory.h"
#include "modules/canbus/vehicle/gem/gem_vehicle_factory.h"
#include "modules/canbus/vehicle/lexus/lexus_vehicle_factory.h"
#include "modules/canbus/vehicle/lincoln/lincoln_vehicle_factory.h"
#include "modules/canbus/vehicle/transit/transit_vehicle_factory.h"
#include "modules/canbus/vehicle/wey/wey_vehicle_factory.h"
#include "modules/canbus/vehicle/zhongyun/zhongyun_vehicle_factory.h"

namespace apollo {
namespace canbus {

void VehicleFactory::RegisterVehicleFactory() {
  Register(apollo::common::LINCOLN_MKZ, []() -> AbstractVehicleFactory * {
    return new LincolnVehicleFactory();
  });
  Register(apollo::common::GEM, []() -> AbstractVehicleFactory * {
    return new GemVehicleFactory();
  });
  Register(apollo::common::LEXUS, []() -> AbstractVehicleFactory * {
    return new LexusVehicleFactory();
  });
  Register(apollo::common::TRANSIT, []() -> AbstractVehicleFactory * {
    return new TransitVehicleFactory();
  });
  Register(apollo::common::GE3, []() -> AbstractVehicleFactory * {
    return new Ge3VehicleFactory();
  });
  Register(apollo::common::WEY, []() -> AbstractVehicleFactory * {
    return new WeyVehicleFactory();
  });
  Register(apollo::common::ZHONGYUN, []() -> AbstractVehicleFactory * {
    return new ZhongyunVehicleFactory();
  });
  Register(apollo::common::CH,
           []() -> AbstractVehicleFactory * { return new ChVehicleFactory(); });
}

std::unique_ptr<AbstractVehicleFactory> VehicleFactory::CreateVehicle(
    const VehicleParameter &vehicle_parameter) {
  auto abstract_factory = CreateObject(vehicle_parameter.brand());
  if (!abstract_factory) {
    AERROR << "failed to create vehicle factory with "
           << vehicle_parameter.DebugString();
  } else {
    abstract_factory->SetVehicleParameter(vehicle_parameter);
    AINFO << "successfully created vehicle factory with "
          << vehicle_parameter.DebugString();
  }
  return abstract_factory;
}

}  // namespace canbus
}  // namespace apollo
