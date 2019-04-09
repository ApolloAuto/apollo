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

#include "modules/canbus/vehicle/wey/wey_vehicle_factory.h"
#include "gtest/gtest.h"
#include "modules/canbus/proto/vehicle_parameter.pb.h"

namespace apollo {
namespace canbus {

class WeyVehicleFactoryTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    VehicleParameter parameter;
    parameter.set_brand(apollo::common::WEY);
    wey_factory_.SetVehicleParameter(parameter);
  }
  virtual void TearDown() {}

 protected:
  WeyVehicleFactory wey_factory_;
};

TEST_F(WeyVehicleFactoryTest, InitVehicleController) {
  EXPECT_NE(wey_factory_.CreateVehicleController(), nullptr);
}

TEST_F(WeyVehicleFactoryTest, InitMessageManager) {
  EXPECT_NE(wey_factory_.CreateMessageManager(), nullptr);
}

}  // namespace canbus
}  // namespace apollo
