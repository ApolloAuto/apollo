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

#include "modules/common/configs/vehicle_config_helper.h"

#include <cmath>
#include <string>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace apollo {
namespace common {

class VehicleConfigHelperTest : public ::testing::Test {
 public:
  void SetUp() {
    config_.mutable_vehicle_param()->set_front_edge_to_center(2.0);
    config_.mutable_vehicle_param()->set_back_edge_to_center(1.0);
    config_.mutable_vehicle_param()->set_left_edge_to_center(1.0);
    config_.mutable_vehicle_param()->set_right_edge_to_center(1.0);
    config_.mutable_vehicle_param()->set_min_turn_radius(5.0);
  }

  VehicleConfig config_;
};

TEST_F(VehicleConfigHelperTest, MinSafeTurnRadius) {
  config_.mutable_vehicle_param()->set_right_edge_to_center(0.5);
  config_.mutable_vehicle_param()->set_front_edge_to_center(3.0);
  VehicleConfigHelper::Init(config_);
  auto min_radius = VehicleConfigHelper::MinSafeTurnRadius();
  EXPECT_DOUBLE_EQ(min_radius, sqrt(36.0 + 9.0));
}

}  // namespace common
}  // namespace apollo
