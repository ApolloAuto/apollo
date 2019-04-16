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

#include "modules/canbus/vehicle/lincoln/lincoln_vehicle_factory.h"

#include "gtest/gtest.h"

#include "modules/canbus/proto/vehicle_parameter.pb.h"

namespace apollo {
namespace canbus {

class LincolnVehicleFactoryTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    VehicleParameter parameter;
    parameter.set_brand(apollo::common::LINCOLN_MKZ);
    lincoln_factory_.SetVehicleParameter(parameter);
  }
  virtual void TearDown() {}

 protected:
  LincolnVehicleFactory lincoln_factory_;
};

TEST_F(LincolnVehicleFactoryTest, InitVehicleController) {
  EXPECT_NE(lincoln_factory_.CreateVehicleController(), nullptr);
}

TEST_F(LincolnVehicleFactoryTest, InitMessageManager) {
  EXPECT_NE(lincoln_factory_.CreateMessageManager(), nullptr);
}

}  // namespace canbus
}  // namespace apollo
