/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "gtest/gtest.h"

#include "modules/canbus/proto/vehicle_parameter.pb.h"

namespace apollo {
namespace canbus {

class VehicleFactoryTest : public ::testing::Test {
 public:
  virtual void SetUp() { factory_.RegisterVehicleFactory(); }

 protected:
  VehicleFactory factory_;
};

TEST_F(VehicleFactoryTest, CreateVehicle) {
  VehicleParameter parameter;

  parameter.set_brand(apollo::common::GEM);
  EXPECT_NE(factory_.CreateVehicle(parameter), nullptr);

  parameter.set_brand(apollo::common::LINCOLN_MKZ);
  EXPECT_NE(factory_.CreateVehicle(parameter), nullptr);

  parameter.set_brand(apollo::common::GE3);
  EXPECT_NE(factory_.CreateVehicle(parameter), nullptr);

  parameter.set_brand(apollo::common::WEY);
  EXPECT_NE(factory_.CreateVehicle(parameter), nullptr);

  parameter.set_brand(apollo::common::ZHONGYUN);
  EXPECT_NE(factory_.CreateVehicle(parameter), nullptr);

  parameter.set_brand(apollo::common::CH);
  EXPECT_NE(factory_.CreateVehicle(parameter), nullptr);
}

}  // namespace canbus
}  // namespace apollo
