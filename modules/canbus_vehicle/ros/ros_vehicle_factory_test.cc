// Copyright 2025 daohu527@gmail.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//  Created Date: 2025-01-16
//  Author: daohu527

#include "modules/canbus_vehicle/ros/ros_vehicle_factory.h"

#include "gtest/gtest.h"
#include "modules/canbus/proto/vehicle_parameter.pb.h"

namespace apollo {
namespace canbus {

class RosVehicleFactoryTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    VehicleParameter parameter;
    parameter.set_brand(apollo::common::ROS);
    ros_factory_.SetVehicleParameter(parameter);
  }
  virtual void TearDown() {}

 protected:
  RosVehicleFactory ros_factory_;
};

TEST_F(RosVehicleFactoryTest, InitVehicleController) {
  EXPECT_NE(ros_factory_.CreateVehicleController(), nullptr);
}

TEST_F(RosVehicleFactoryTest, InitMessageManager) {
  EXPECT_NE(ros_factory_.CreateMessageManager(), nullptr);
}

}  // namespace canbus
}  // namespace apollo
