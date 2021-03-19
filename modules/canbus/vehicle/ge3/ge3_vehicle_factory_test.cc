/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/vehicle/ge3/ge3_vehicle_factory.h"
#include "gtest/gtest.h"
#include "modules/canbus/proto/vehicle_parameter.pb.h"

namespace apollo {
namespace canbus {

class Ge3VehicleFactoryTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    VehicleParameter parameter;
    parameter.set_brand(apollo::common::GE3);
    ge3_factory_.SetVehicleParameter(parameter);
  }
  virtual void TearDown() {}

 protected:
  Ge3VehicleFactory ge3_factory_;
};

TEST_F(Ge3VehicleFactoryTest, InitVehicleController) {
  EXPECT_NE(ge3_factory_.CreateVehicleController(), nullptr);
}

TEST_F(Ge3VehicleFactoryTest, InitMessageManager) {
  EXPECT_NE(ge3_factory_.CreateMessageManager(), nullptr);
}

}  // namespace canbus
}  // namespace apollo
